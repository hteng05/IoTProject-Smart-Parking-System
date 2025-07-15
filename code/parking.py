from flask import Flask, render_template, request, jsonify, redirect, url_for, flash
import serial
import json
import threading
import time
import pymysql
from datetime import datetime
import os
import logging
from collections import deque

# Configure your app logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Suppress default Flask/Werkzeug access logs
logging.getLogger('werkzeug').setLevel(logging.ERROR)


# Flask application setup
app = Flask(__name__)
app.secret_key = os.urandom(24)

# Configure database connection
DB_CONFIG = {
    'host': 'localhost',
    'user': '', # Set your MySQL username
    'password': '', # Set your MySQL password
    'database': '' # Set your MySQL database name
}

# Serial communication configuration
SERIAL_PORT = '' # Update with your actual serial port (usually /dev/cu.wchusbserial* on macOS)
SERIAL_BAUD_RATE = 115200
SERIAL_TIMEOUT = 1

# Global variables
serial_conn = None
serial_data_buffer = deque(maxlen=100)  # Store recent serial messages
system_status = {
    'gate_open': False,
    'available_slots': 2,
    'wifi_connected': False,
    'timestamp': None
}
slots_status = [
    {'id': 1, 'booked': False, 'occupied': False, 'plate': None},
    {'id': 2, 'booked': False, 'occupied': False, 'plate': None}
]


# ========================
# Database Functions
# ========================

def initialize_database():
    """Initialize database tables if they don't exist"""
    try:
        conn = pymysql.connect(**DB_CONFIG)
        cursor = conn.cursor()
        
        # Create slots table
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS slots (
                id INT PRIMARY KEY,
                status VARCHAR(20) NOT NULL DEFAULT 'free',
                plate VARCHAR(20) NULL
            )
        """)
        
        # Create plates table for detected plates
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS plates (
                id INT AUTO_INCREMENT PRIMARY KEY,
                plate_number VARCHAR(20) NOT NULL,
                timestamp DATETIME NOT NULL,
                slot_assigned INT NULL,
                status VARCHAR(20) NOT NULL DEFAULT 'detected'
            )
        """)
        
        # Create log table for system events
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS system_logs (
                id INT AUTO_INCREMENT PRIMARY KEY,
                event_type VARCHAR(50) NOT NULL,
                event_data TEXT NULL,
                timestamp DATETIME NOT NULL
            )
        """)
        
        # Insert default slots if not exist
        cursor.execute("SELECT COUNT(*) FROM slots")
        if cursor.fetchone()[0] < 2:
            cursor.execute("DELETE FROM slots")  # Clear existing if partial
            cursor.execute("INSERT INTO slots (id, status) VALUES (1, 'free')")
            cursor.execute("INSERT INTO slots (id, status) VALUES (2, 'free')")
        
        conn.commit()
        logger.info("Database initialized successfully")
        
        # Load initial slot status
        load_slots_from_database()
    except Exception as e:
        logger.error(f"Database initialization error: {str(e)}")
    finally:
        if conn:
            conn.close()
    
# This function logs system events to the database: system_logs       
def log_system_event(event_type, event_data=None):
    """Log system event to database"""
    try:
        conn = pymysql.connect(**DB_CONFIG)
        cursor = conn.cursor()
        
        timestamp = datetime.now()
        cursor.execute(
            "INSERT INTO system_logs (event_type, event_data, timestamp) VALUES (%s, %s, %s)",
            (event_type, event_data, timestamp)
        )
            
        conn.commit()
    except Exception as e:
        logger.error(f"Failed to log system event: {str(e)}")
    finally:
        if conn:
            conn.close()

def load_slots_from_database():
    """Load slot information from database"""
    try:
        conn = pymysql.connect(**DB_CONFIG)
        cursor = conn.cursor(pymysql.cursors.DictCursor)
        cursor.execute("SELECT * FROM slots")
        db_slots = cursor.fetchall()
        
        # Update global slots status
        for db_slot in db_slots:
            slot_idx = db_slot['id'] - 1
            if slot_idx >= 0 and slot_idx < len(slots_status):
                slots_status[slot_idx]['booked'] = db_slot['status'] == 'booked'
                slots_status[slot_idx]['plate'] = db_slot['plate']
                
        # Update Arduino about current slot status
        send_slot_info_to_arduino()
    except Exception as e:
        logger.error(f"Failed to load slots from database: {str(e)}")
    finally:
        if conn:
            conn.close()
            
# This function updates the slot status in the database and also updates the global state
def update_slot_in_database(slot_id, status, plate=None):
    """Update slot status in database"""
    try:
        conn = pymysql.connect(**DB_CONFIG)
        cursor = conn.cursor()
        
        # Handle different statuses properly
        if status == "free":
            cursor.execute(
                "UPDATE slots SET status = %s, plate = NULL WHERE id = %s",
                (status, slot_id)
            )
        elif status == "occupied":
            # Check if this slot was already booked - if so, maintain its status
            cursor.execute("SELECT status, plate FROM slots WHERE id = %s", (slot_id,))
                # It's an unbooked slot being occupied
            cursor.execute(
                "UPDATE slots SET status = %s, plate = %s WHERE id = %s", 
                (status, plate, slot_id)
            )
        else:
            # Booked or other status
            cursor.execute(
                "UPDATE slots SET status = %s, plate = %s WHERE id = %s", 
                (status, plate, slot_id)
            )
            
        conn.commit()
        # Update global state
        slots_status[slot_id-1]['booked'] = (status == 'booked')
        slots_status[slot_id-1]['occupied'] = (status == 'occupied' or 
                                              (status == 'booked' and slots_status[slot_id-1]['occupied']))
        slots_status[slot_id-1]['plate'] = plate
        
        # Notify Arduino about the change
        send_slot_info_to_arduino()
        
        return True
    except Exception as e:
        logger.error(f"Database update error for slot {slot_id}: {str(e)}")
        return False
    finally:
        if conn:
            conn.close()

# This function stores detected plates in the database
def store_detected_plate(plate_number, slot_assigned=None):
    """Store a detected license plate in the database"""
    try:
        conn = pymysql.connect(**DB_CONFIG)
        cursor = conn.cursor()
        
        timestamp = datetime.now()
        status = 'assigned' if slot_assigned else 'detected'
        
        cursor.execute(
            "INSERT INTO plates (plate_number, timestamp, slot_assigned, status) VALUES (%s, %s, %s, %s)",
            (plate_number, timestamp, slot_assigned, status)
        )
        
        conn.commit()
        return True
    except Exception as e:
        logger.error(f"Failed to store detected plate: {str(e)}")
        return False
    finally:
        if conn:
            conn.close()

#THIS IS REALLY IMPORTANT PART: this function checks if a plate has a reservation and returns the slot if found
def check_plate_reservation(plate_number):
    """Check if a plate has a reservation and return slot if found"""
    conn = None

    try:
        conn = pymysql.connect(**DB_CONFIG)
        cursor = conn.cursor(pymysql.cursors.DictCursor)
        
        cursor.execute(
            "SELECT id FROM slots WHERE status = 'booked' AND plate = %s",
            (plate_number,)
        )
        result = cursor.fetchone()
        
        if result:
            return result['id']
        return None
    except Exception as e:
        logger.error(f"Error checking plate reservation: {str(e)}")
        return None
    finally:
        if conn:
            conn.close()

# This function finds an available parking slot
def find_available_slot():
    """Find an available parking slot"""
    try:
        conn = pymysql.connect(**DB_CONFIG)
        cursor = conn.cursor(pymysql.cursors.DictCursor)
        
        cursor.execute("SELECT id FROM slots WHERE status = 'free' LIMIT 1")
        result = cursor.fetchone()
        
        if result:
            return result['id']
        return None
    except Exception as e:
        logger.error(f"Error finding available slot: {str(e)}")
        return None
    finally:
        if conn:
            conn.close()

# This function retrieves recent logs from the database and then uploads them to the web interface
def get_recent_logs(limit=20):
    """Get recent system logs"""
    try:
        conn = pymysql.connect(**DB_CONFIG)
        cursor = conn.cursor(pymysql.cursors.DictCursor)
        
        cursor.execute(
            "SELECT * FROM system_logs ORDER BY timestamp DESC LIMIT %s",
            (limit,)
        )
        
        return cursor.fetchall()
    except Exception as e:
        logger.error(f"Error retrieving logs: {str(e)}")
        return []
    finally:
        if conn:
            conn.close()

# This function retrieves recent detected plates from the database and then uploads them to the web interface
def get_recent_plates(limit=20):
    """Get recently detected plates"""
    try:
        conn = pymysql.connect(**DB_CONFIG)
        cursor = conn.cursor(pymysql.cursors.DictCursor)
        
        cursor.execute(
            "SELECT * FROM plates ORDER BY timestamp DESC LIMIT %s",
            (limit,)
        )
        
        return cursor.fetchall()
    except Exception as e:
        logger.error(f"Error retrieving recent plates: {str(e)}")
        return []
    finally:
        if conn:
            conn.close()


# ========================
# Serial Communication
# ========================

def initialize_serial():
    """Initialize serial connection to Arduino"""
    global serial_conn
    try:
        serial_conn = serial.Serial(SERIAL_PORT, SERIAL_BAUD_RATE, timeout=SERIAL_TIMEOUT)
        logger.info(f"Serial connection established on {SERIAL_PORT}")
        return True
    except Exception as e:
        logger.error(f"Failed to initialize serial connection: {str(e)}")
        return False

# This function sends commands to Arduino and handles the response
def send_to_arduino(command):
    """Send a command to Arduino through serial connection"""
    global serial_conn
    try:
        if serial_conn and serial_conn.is_open:
            # Add newline to end command
            if not command.endswith('\n'):
                command += '\n'
            serial_conn.write(command.encode())
            serial_conn.flush()
            logger.info(f"Sent to Arduino: {command.strip()}")
            return True
        else:
            logger.error("Serial connection not available")
            return False
    except Exception as e:
        logger.error(f"Failed to send command to Arduino: {str(e)}")
        return False


def send_slot_info_to_arduino():
    """Send current slot information to Arduino"""
    try:
        # Create JSON structure for Arduino
        slots_data = {
            "slots": [
                {
                    "id": status['id'],
                    "status": "booked" if status['booked'] else "free",
                    "plate": status['plate'] if status['plate'] else ""
                }
                for status in slots_status
            ]
        }
        
        # Send command with JSON data
        command = f"UPDATE_SLOT_INFO{json.dumps(slots_data)}"
        send_to_arduino(command)
        return True
    except Exception as e:
        logger.error(f"Failed to send slot info to Arduino: {str(e)}")
        return False


def read_from_arduino():
    """Read data from Arduino"""
    global serial_conn, system_status, slots_status, serial_data_buffer
    
    try:
        if serial_conn and serial_conn.is_open:
            if serial_conn.in_waiting > 0:
                line = serial_conn.readline().decode('utf-8').strip()
                if line:
                    # Add to buffer
                    serial_data_buffer.append(f"{datetime.now().strftime('%H:%M:%S')} - {line}")
                    process_arduino_message(line)
                return line
        return None
    except Exception as e:
        logger.error(f"Error reading from Arduino: {str(e)}")
        return None

# This function processes messages received from Arduino and updates the system state
def process_arduino_message(message):
    """Process messages received from Arduino"""
    global system_status, slots_status
    
    try:
        # Add debugging - print all messages being processed
        print(f"{message}")
        
        # Handle different message types
        if message.startswith("DETECTED_PLATE:"):
            plate = message.split(":")[1]
            log_system_event("PLATE_READ", plate)
            # Check if plate has a booking
            reserved_slot = check_plate_reservation(plate)

            if reserved_slot:
                # It's a reserved plate — log as assigned
                store_detected_plate(plate, slot_assigned=reserved_slot)
            else:
                # No reservation — try to auto-assign a free slot
                free_slot = find_available_slot()
                
                if free_slot:
                    # Update DB to assign this free slot to the plate
                    store_detected_plate(plate, slot_assigned=free_slot)
                else:
                    # No free slot available — just log the plate as detected
                    store_detected_plate(plate)
            
        elif message.startswith("GATE_STATUS:"):
            status = message.split(":")[1]
            system_status['gate_open'] = (status == "OPENED")
            log_system_event("GATE_STATUS", status)
            
        elif message.startswith("VEHICLE:DETECTED_AT_ENTRANCE"):
            log_system_event("VEHICLE_DETECTED", "Vehicle at entrance")
            
        elif message.startswith("WRONG_PARKING:"):
            slot = message.split(":")[1]
            log_system_event("WRONG_PARKING", f"Wrong parking in {slot}")
        
        elif message.strip() == "PARKING:FULL":
            log_system_event("PARKING_FULL", "No parking slots available")
            
        elif message.startswith("SLOT_ASSIGNED:"):
            parts = message.split(":")
            assignment_type = parts[1]
            slot_num = int(parts[2])
            
            if assignment_type == "RESERVED":
                log_system_event("SLOT_ASSIGNED", f"Reserved slot {slot_num} assigned")
            else:
                log_system_event("SLOT_ASSIGNED", f"Available slot {slot_num} assigned")
            
        elif message.startswith("SLOT1:") or message.startswith("SLOT2:"):
            slot_num = int(message[4:5])
            status = message.split(":")[1]
            
            # Update occupancy but preserve booking status
            if status == "OCCUPIED":
                # Get current slot status from database
                conn = pymysql.connect(**DB_CONFIG)
                cursor = conn.cursor(pymysql.cursors.DictCursor)
                cursor.execute("SELECT status, plate FROM slots WHERE id = %s", (slot_num,))
                current = cursor.fetchone()
                conn.close()
                
                if current and current['status'] == "booked":
                    # It's a booked slot that is now physically occupied
                    slots_status[slot_num-1]['occupied'] = True

                    # Update DB: preserve booking but mark as occupied
                    update_slot_in_database(slot_num, "occupied", "BookedPlate")
                    log_system_event("SLOT_OCCUPIED", f"Booked slot {slot_num} is now occupied")

                else:
                    # Walk-in car (no booking)
                    slots_status[slot_num-1]['occupied'] = True
                    # Update DB: mark slot as occupied and store detected plate if needed
                    update_slot_in_database(slot_num, "occupied", "UnbookedPlate")
                    log_system_event("SLOT_OCCUPIED", f"Unbooked slot {slot_num} is now occupied")

            
            elif status == "VACATED":
                slots_status[slot_num-1]['occupied'] = False

                # Directly update DB to clear booking & free the slot
                update_slot_in_database(slot_num, "free")
                # log the removal
                log_system_event("SLOT_FREED", f"Slot {slot_num} marked as free (vacated)")
        
        
    except Exception as e:
        logger.error(f"Error processing Arduino message '{message}': {str(e)}")


def serial_monitor_thread():
    """Background thread to monitor serial communication"""
    while True:
        try:
            read_from_arduino()
            time.sleep(0.1)  # Small delay to prevent CPU hogging
        except Exception as e:
            logger.error(f"Serial monitor thread error: {str(e)}")
            time.sleep(1)  # Longer delay on error


# ========================
# Flask Routes 
# ========================

@app.route('/')
def index():
    """Main dashboard page"""
    return render_template('index.html', 
                          slots=slots_status, 
                          system=system_status, 
                          serial_messages=list(serial_data_buffer))


@app.route('/slots')
def slots_page():
    """Slot management page"""
    return render_template('slots.html', slots=slots_status)


@app.route('/logs')
def logs_page():
    """System logs page"""
    logs = get_recent_logs(50)
    plates = get_recent_plates(20)
    return render_template('logs.html', logs=logs, plates=plates)


@app.route('/book_slot', methods=['POST'])
def book_slot():
    """Book a parking slot"""
    slot_id = int(request.form.get('slot_id'))
    plate_number = request.form.get('plate_number')
    
    if not plate_number:
        flash('License plate number is required', 'error')
        return redirect(url_for('index'))
    
    # Check if the plate already has a booking
    existing_slot = check_plate_reservation(plate_number)
    if existing_slot:
        flash(f'This plate already has a booking for slot {existing_slot}', 'error')
        return redirect(url_for('index'))
    
    # Update database
    success = update_slot_in_database(slot_id, 'booked', plate_number)
    
    if success:
        flash(f'Slot {slot_id} successfully booked for plate {plate_number}', 'success')
    else:
        flash('Failed to book slot. Please try again.', 'error')
    
    return redirect(url_for('index'))


@app.route('/cancel_booking', methods=['POST'])
def cancel_booking():
    """Cancel a slot booking"""
    slot_id = int(request.form.get('slot_id'))
    
    # Update database
    success = update_slot_in_database(slot_id, 'free')
    
    if success:
        flash(f'Booking for slot {slot_id} has been cancelled', 'success')
    else:
        flash('Failed to cancel booking. Please try again.', 'error')
    
    return redirect(url_for('index'))


@app.route('/api/system_status')
def get_system_status():
    try:
        conn = pymysql.connect(**DB_CONFIG)
        cursor = conn.cursor(pymysql.cursors.DictCursor)

        # Count slots that are actually free
        cursor.execute("SELECT COUNT(*) AS available FROM slots WHERE status = 'free'")
        available = cursor.fetchone()['available']

        # Fetch current full slot status
        cursor.execute("SELECT * FROM slots")
        all_slots = cursor.fetchall()

        conn.close()

        return jsonify({
            "system": {
                "available_slots": available,
                "wifi_connected": True,  # set based on your actual check
                "timestamp": time.time()
            },
            "slots": all_slots
        })

    except Exception as e:
        logger.error(f"System status error: {e}")
        return jsonify({
            "system": {
                "available_slots": 0,
                "wifi_connected": False,
                "timestamp": time.time()
            },
            "slots": []
        })


@app.route('/api/serial_messages')
def api_serial_messages():
    """API endpoint for recent serial messages"""
    return jsonify(list(serial_data_buffer))


@app.route('/api/send_command', methods=['POST'])
def api_send_command():
    """API endpoint to send commands to Arduino"""
    command = request.form.get('command')
    
    if not command:
        return jsonify({'success': False, 'message': 'No command provided'})
    
    success = send_to_arduino(command)
    return jsonify({'success': success})


@app.route('/arduino/open_gate')
def arduino_open_gate():
    """Direct command to open gate"""
    success = send_to_arduino("OPEN_GATE")
    if success:
        flash('Command sent to open gate', 'success')
    else:
        flash('Failed to send open gate command', 'error')
    return redirect(url_for('index'))


@app.route('/arduino/close_gate')
def arduino_close_gate():
    """Direct command to close gate"""
    success = send_to_arduino("CLOSE_GATE")
    if success:
        flash('Command sent to close gate', 'success')
    else:
        flash('Failed to send close gate command', 'error')
    return redirect(url_for('index'))


@app.route('/arduino/toggle_led', methods=['POST'])
def arduino_toggle_led():
    """Toggle an LED on Arduino"""
    pin = request.form.get('pin')
    if not pin:
        flash('No LED pin specified', 'error')
        return redirect(url_for('index'))
    
    success = send_to_arduino(f"TOGGLE_LED:{pin}")
    if success:
        flash(f'Command sent to toggle LED on pin {pin}', 'success')
    else:
        flash('Failed to send toggle LED command', 'error')
    return redirect(url_for('index'))


@app.route('/arduino/buzzer', methods=['POST'])
def arduino_buzzer():
    """Activate buzzer on Arduino"""
    duration = request.form.get('duration', '1000')
    success = send_to_arduino(f"ACTIVATE_BUZZER:{duration}")
    if success:
        flash(f'Command sent to activate buzzer for {duration}ms', 'success')
    else:
        flash('Failed to send buzzer command', 'error')
    return redirect(url_for('index'))


@app.route('/arduino/get_status')
def arduino_get_status():
    """Request status update from Arduino"""
    success = send_to_arduino("GET_STATUS")
    if success:
        flash('Status update requested from Arduino', 'success')
    else:
        flash('Failed to request status from Arduino', 'error')
    return redirect(url_for('index'))


@app.route('/arduino/reset')
def arduino_reset():
    """Reset Arduino"""
    success = send_to_arduino("RESET")
    if success:
        flash('Reset command sent to Arduino', 'success')
    else:
        flash('Failed to send reset command', 'error')
    return redirect(url_for('index'))


# ========================
# Application Initialization
# ========================

def initialize_app():
    """Initialize the application - database and serial connection"""
    # Initialize database
    initialize_database()
    
    # Try to initialize serial connection
    serial_success = initialize_serial()
    if not serial_success:
        logger.warning("Serial connection failed, will retry in background")
    
    # Start serial monitor thread
    thread = threading.Thread(target=serial_monitor_thread, daemon=True)
    thread.start()
    
    # Request initial status from Arduino if connection successful
    if serial_success:
        send_to_arduino("GET_STATUS")


if __name__ == '__main__':
    # Initialize app components
    initialize_app()
    
    # Run Flask app
    app.run(host='0.0.0.0', port=8080, debug=True, use_reloader=False)