#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "esp_camera.h"
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>

// WiFi credentials for license plate API connection
const char* ssid = "Your_SSID"; // Replace with your WiFi SSID
const char* password = "Your_PASSWORD"; // Replace with your WiFi password

// API Server for License Plate Detection
String serverName = "www.circuitdigest.cloud";
String serverPath = "/readnumberplate";
const int serverPort = 443;
String apiKey = "YOURAPIKEY"; // Your API key for authentication

// Initialize secure client for HTTPS API calls
WiFiClientSecure client;

// Camera GPIO pins for ESP32-S3 (correctly mapped from the pinout diagram)
//the settings will depend on the specific ESP32-S3 camera module you are using, please refer to the module's documentation for exact pin mappings.
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 15
#define SIOD_GPIO_NUM 4
#define SIOC_GPIO_NUM 5

#define Y2_GPIO_NUM 11
#define Y3_GPIO_NUM 9
#define Y4_GPIO_NUM 8
#define Y5_GPIO_NUM 10
#define Y6_GPIO_NUM 12
#define Y7_GPIO_NUM 18
#define Y8_GPIO_NUM 17
#define Y9_GPIO_NUM 16

#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 7
#define PCLK_GPIO_NUM 13

// NTP Client setup
const char* ntpServer = "pool.ntp.org";
const long utcOffsetInSeconds = 25200;  // UTC+7 (Adjust to your time zone)
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer, utcOffsetInSeconds);

// Define GPIO pins - CORRECTED for ESP32-S3 WROOM
// Entrance sensor and servo
const int gateIrSensorPin = 1;   // IR Sensor at entrance gate 
const int servoPin = 14;         // Servo motor for gate control
const int buzzerPin = 46;        // Buzzer for alerts 

// Slot 1
const int slot1IrSensorPin = 19; // IR Sensor for slot 1 
const int slot1RedLedPin = 20;   // Red LED for slot 1 
const int slot1GreenLedPin = 2;  // Green LED for slot 1 

// Slot 2
const int slot2IrSensorPin = 21; // IR Sensor for slot 2 
const int slot2RedLedPin = 35;   // Red LED for slot 2 
const int slot2GreenLedPin = 36; // Green LED for slot 2 

// Initialize components
Servo gateServo;
const int gateTimeoutPotPin = 3;

// System state variables
String detectedPlate = "";
String imageLink = ""; 
bool isGateOpen = false;
int availableSlots = 2; // Total number of slots
unsigned long gateOpenTime = 0;
unsigned long gateOpenDuration = 5000; // 10 seconds to close gate automatically

// Serial communication parameters
const unsigned long SERIAL_BAUD_RATE = 115200;
const unsigned long SERIAL_TIMEOUT = 50;  // ms
const unsigned long DATA_SEND_INTERVAL = 4000; // ms
unsigned long lastDataSentTime = 0;

// Serial protocol commands
// Commands from edge device
const String CMD_OPEN_GATE = "OPEN_GATE";
const String CMD_CLOSE_GATE = "CLOSE_GATE";
const String CMD_TOGGLE_LED = "TOGGLE_LED";
const String CMD_ACTIVATE_BUZZER = "ACTIVATE_BUZZER";
const String CMD_GET_STATUS = "GET_STATUS";
const String CMD_UPDATE_SLOT_INFO = "UPDATE_SLOT_INFO";
const String CMD_RESET = "RESET";

// Define a buffer for incoming serial data
String serialBuffer = "";

// Slot status
struct SlotStatus {
  bool isBooked;         // Booked on website
  bool isOccupied;       // Physically occupied by a car
  String bookedPlate;    // Plate number that booked this slot
};

SlotStatus slots[2]; // Array for 2 slots (index 0 for slot 1, index 1 for slot 2)

// Function prototypes
bool initCamera();
void connectToWiFi();
void openGate();
void closeGate();
void blinkLED(int pin, int times, int delayTime);
void toggleLED(int ledPin);
void activateBuzzer(int durationMs);
void updateSlotLEDs();
String captureAndRecognizePlate();
int findMatchingSlot(String plateNumber);
int findAvailableSlot();
bool checkWrongParking(int slotIndex);
void sendSystemStatus();
void parseSlotInfo(String infoString);
void processSerialCommand(String command);
void checkSerialInput();
String extractJsonStringValue(const String& jsonString, const String& key);
int count = 0;        // Counter for image uploads

// Function to extract a JSON string value by key (from API response)
String extractJsonStringValue(const String& jsonString, const String& key) {
  int keyIndex = jsonString.indexOf(key);
  if (keyIndex == -1) {
    return "";
  }

  int startIndex = jsonString.indexOf(':', keyIndex) + 2;
  int endIndex = jsonString.indexOf('"', startIndex);

  if (startIndex == -1 || endIndex == -1) {
    return "";
  }

  return jsonString.substring(startIndex, endIndex);
}

// Initialize the camera
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Adjust frame size and quality based on PSRAM availability
    if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 5;
    config.fb_count = 2; 
    Serial.println("PSRAM found");
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM; 
    Serial.println("PSRAM not found");
  }
  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return false;
  }
  Serial.println("Camera initialized successfully");
  sensor_t * s = esp_camera_sensor_get();
  s->set_brightness(s, 2);
  s->set_saturation(s, -1);
  return true;
}


// Function to connect to WiFi
void connectToWiFi() {
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed");
  }
}

// Function to open the gate
void openGate() {
  gateServo.write(0); // Position for open gate
  isGateOpen = true;
  
  // Read potentiometer to set timeout duration
  int potValue = analogRead(gateTimeoutPotPin);
  // Map to a reasonable range
  unsigned long timeoutDuration = map(potValue, 0, 4095, 500, 5000);
  
  gateOpenTime = millis();
  gateOpenDuration = timeoutDuration; 
  
  Serial.print("GATE_STATUS:OPENED,TIMEOUT:");
  Serial.println(timeoutDuration);
}

// Function to close the gate
void closeGate() {
  gateServo.write(90); // Position for closed gate
  isGateOpen = false;
  Serial.println("GATE_STATUS:CLOSED");
}

// Function to blink LED
void blinkLED(int pin, int times, int delayTime) {
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(delayTime);
    digitalWrite(pin, LOW);
    delay(delayTime);
  }
}

// Function to toggle LED state
void toggleLED(int ledPin) {
  bool currentState = digitalRead(ledPin);
  digitalWrite(ledPin, !currentState);
  Serial.print("LED_TOGGLED:");
  Serial.println(ledPin);
}

// Function to activate buzzer
void activateBuzzer(int durationMs) {
  digitalWrite(buzzerPin, HIGH);
  Serial.println("BUZZER:ON");
  delay(durationMs);
  digitalWrite(buzzerPin, LOW);
  Serial.println("BUZZER:OFF");
}

// Function to update slot LEDs based on status
void updateSlotLEDs() {
  // Slot 1
  digitalWrite(slot1RedLedPin, slots[0].isBooked ? HIGH : LOW);
  // digitalWrite(slot1GreenLedPin, (slots[0].isOccupied) ? HIGH : LOW);
  
  // Slot 2
  digitalWrite(slot2RedLedPin, slots[1].isBooked ? HIGH : LOW);
  // digitalWrite(slot2GreenLedPin, (slots[1].isOccupied && !slots[1].isBooked) ? HIGH : LOW);
  
  // Send LED status to edge device through serial
  Serial.print("LED_STATUS:SLOT1_RED:");
  Serial.print(digitalRead(slot1RedLedPin));
  Serial.print(",SLOT1_GREEN:");
  Serial.print(digitalRead(slot1GreenLedPin));
  Serial.print(",SLOT2_RED:");
  Serial.print(digitalRead(slot2RedLedPin));
  Serial.print(",SLOT2_GREEN:");
  Serial.println(digitalRead(slot2GreenLedPin));
}

// Function to capture image and get license plate number
String captureAndRecognizePlate() {
  camera_fb_t* fb = NULL;
  // Take a photo
  Serial.println("CAMERA:CAPTURING");

  delay(300);
  fb = esp_camera_fb_get();
  delay(300);

  if (!fb) {
    Serial.println("CAMERA:FAILED");
    return "";
  }
  
  Serial.println("Connecting to server:" + serverName);
  client.setInsecure(); // Skip certificate validation for simplicity
  
  if (client.connect(serverName.c_str(), serverPort)) {
    Serial.println("SERVER:CONNECTED");
    
    // Prepare file name for the image
    count++;
    Serial.println(count);
    String filename = apiKey + ".jpeg";
    
    // Prepare HTTP POST request
    String head = "--CircuitDigest\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"" + filename + "\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--CircuitDigest--\r\n";
    uint32_t imageLen = fb->len;
    uint32_t extraLen = head.length() + tail.length();
    uint32_t totalLen = imageLen + extraLen;
    
    client.println("POST " + serverPath + " HTTP/1.1");
    client.println("Host: " + serverName);
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=CircuitDigest");
    client.println("Authorization:" + apiKey);
    client.println();
    client.print(head);
    
    // Send the image data in chunks
    uint8_t* fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n = 0; n < fbLen; n += 1024) {
      if (n + 1024 < fbLen) {
        client.write(fbBuf, 1024);
        fbBuf += 1024;
      } else {
        size_t remainder = fbLen % 1024;
        client.write(fbBuf, remainder);
      }
    }
    
    client.print(tail);
    // Release resources
  esp_camera_fb_return(fb);
  activateBuzzer(200);
  Serial.println("Image sent successfully");

    // Wait for response
    String response = "";
    long startTime = millis();
    while (client.connected() && millis() - startTime < 10000) {
      if (client.available()) {
        char c = client.read();
        response += c;
      }
    }
    
    // Extract plate number from response
    String plateNumber = extractJsonStringValue(response, "\"number_plate\"");
    String imageLink = extractJsonStringValue(response, "\"view_image\"");
    
    client.stop();
    Serial.print("PLATE_DETECTED:");
    Serial.println(response);
    return plateNumber; 
  } else {
    Serial.println("SERVER:CONNECTION_FAILED");
    esp_camera_fb_return(fb);
    return "";
  }
}

// Function to find if a detected plate matches any booked slot
int findMatchingSlot(String plateNumber) {
  for (int i = 0; i < 2; i++) {
    // Check if this slot is booked with the matching plate
    if (slots[i].isBooked && slots[i].bookedPlate == plateNumber) {
      return i + 1; // Return slot number (1-based)
    }
  }
  return 0; // No matching slot found
}

// Function to find an available slot for an unregistered car
int findAvailableSlot() {
  // Check if any slot is not booked
  for (int i = 0; i < 2; i++) {
    if (!slots[i].isBooked && !slots[i].isOccupied) {
      return i + 1; // Return slot number (1-based)
    }
  }
  return 0; // No available slot
}

// Function to check for wrong parking
bool checkWrongParking(int slotIndex) {
  // If IR sensor detects a car but the slot is booked for another plate
  bool isOccupied = (digitalRead(slotIndex == 0 ? slot1IrSensorPin : slot2IrSensorPin) == LOW);
  
  // If a slot is occupied and booked, but not for the current car
  if (isOccupied) {
    if (slots[slotIndex].isBooked && detectedPlate != slots[slotIndex].bookedPlate) {
      // This slot is booked but not by the current car
      return true;
    }
  }
  return false;
}

// Function to send system status to the edge device
void sendSystemStatus() {
  // Use StaticJsonDocument on the stack instead of heap allocation
  StaticJsonDocument<1024> statusDoc;
  
  // System information
  statusDoc["system"]["gate_open"] = isGateOpen;
  statusDoc["system"]["available_slots"] = availableSlots;
  statusDoc["system"]["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
  statusDoc["system"]["time"] = timeClient.getFormattedTime();
  
  // Slot information
  for (int i = 0; i < 2; i++) {
    JsonObject slot = statusDoc["slots"][i].to<JsonObject>();
    slot["id"] = i + 1;
    slot["booked"] = slots[i].isBooked;
    slot["occupied"] = slots[i].isOccupied;
    slot["plate"] = slots[i].bookedPlate;
  }
  
  // Sensor readings
  statusDoc["sensors"]["gate_ir"] = digitalRead(gateIrSensorPin);
  statusDoc["sensors"]["slot1_ir"] = digitalRead(slot1IrSensorPin);
  statusDoc["sensors"]["slot2_ir"] = digitalRead(slot2IrSensorPin);
  
  // Output directly to serial
  Serial.print("STATUS:");
  serializeJson(statusDoc, Serial);
  Serial.println();
  
  // No need to free memory as we're using a stack-allocated document
}

// Parse slot info received from edge device
void parseSlotInfo(String infoString) {
  // Use StaticJsonDocument on the stack instead of heap allocation
  StaticJsonDocument<1024> doc;
  
  DeserializationError error = deserializeJson(doc, infoString); // Removed * operator
  if (error) {
    Serial.print("JSON parse failed: ");
    Serial.println(error.c_str());
    return; // Removed delete as we're using stack allocation
  }
  
  JsonArray slotsArray = doc["slots"].as<JsonArray>(); // Removed * operator
  int i = 0;
  for (JsonObject slot : slotsArray) {
    if (i < 2) { // Only handle 2 slots
      const char* status = slot["status"];
      const char* plate = slot["plate"];
      
      slots[i].isBooked = (String(status) == "booked");
      slots[i].bookedPlate = plate ? String(plate) : "";
      i++;
    }
  }
  
  updateSlotLEDs();
  Serial.println("SLOT_INFO:UPDATED");

}


// Process commands received from edge device
void processSerialCommand(String command) {
  // Log the received command
  Serial.print("COMMAND_RECEIVED:");
  Serial.println(command);
  
  // Parse the command
  if (command.startsWith(CMD_OPEN_GATE)) {
    openGate();
  } 
  else if (command.startsWith(CMD_CLOSE_GATE)) {
    closeGate();
  } 
  else if (command.startsWith(CMD_TOGGLE_LED)) {
    // Format: TOGGLE_LED:PIN
    int pinIndex = command.indexOf(':');
    if (pinIndex != -1) {
      String pinStr = command.substring(pinIndex + 1);
      int pin = pinStr.toInt();
      toggleLED(pin);
    }
  } 
  else if (command.startsWith(CMD_ACTIVATE_BUZZER)) {
    // Format: ACTIVATE_BUZZER:DURATION
    int durationIndex = command.indexOf(':');
    if (durationIndex != -1) {
      String durationStr = command.substring(durationIndex + 1);
      int duration = durationStr.toInt();
      activateBuzzer(duration);
    }
  } 
  else if (command.equals(CMD_GET_STATUS)) {
    sendSystemStatus();
  } 
  else if (command.startsWith(CMD_UPDATE_SLOT_INFO)) {
    // Format: UPDATE_SLOT_INFO:{json_data}
    int dataIndex = command.indexOf('{');
    if (dataIndex != -1) {
      String slotData = command.substring(dataIndex);
      parseSlotInfo(slotData);
    }
  } 
  else if (command.equals(CMD_RESET)) {
    Serial.println("SYSTEM:RESETTING");
    ESP.restart();
  }
  else {
    Serial.println("COMMAND:UNKNOWN");
  }
}

// Check for incoming serial commands
void checkSerialInput() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    // If newline or carriage return, process the command
    if (inChar == '\n' || inChar == '\r') {
      if (serialBuffer.length() > 0) {
        processSerialCommand(serialBuffer);
        serialBuffer = ""; // Clear buffer after processing
      }
    } else {
      serialBuffer += inChar; // Add character to buffer
    }
  }
}

void setup() {
  // Initialize serial communication with edge device
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.setTimeout(SERIAL_TIMEOUT);
  
  Serial.println("SYSTEM:INITIALIZING");
  
  // Initialize GPIO pins
  pinMode(gateIrSensorPin, INPUT);
  pinMode(slot1IrSensorPin, INPUT);
  pinMode(slot2IrSensorPin, INPUT);
  pinMode(slot1RedLedPin, OUTPUT);
  pinMode(slot1GreenLedPin, OUTPUT);
  pinMode(slot2RedLedPin, OUTPUT);
  pinMode(slot2GreenLedPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  
  // Set default state for outputs
  digitalWrite(slot1RedLedPin, LOW);
  digitalWrite(slot1GreenLedPin, LOW);
  digitalWrite(slot2RedLedPin, LOW);
  digitalWrite(slot2GreenLedPin, LOW);
  digitalWrite(buzzerPin, LOW);

  
  // Initialize servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  gateServo.setPeriodHertz(50);
  gateServo.attach(servoPin, 500, 2400);
  closeGate();
  
  // Connect to WiFi
  connectToWiFi();
  
  // Initialize NTP
  timeClient.begin();
  
  // Initialize camera
  if (!initCamera()) {
    Serial.println("CAMERA:INIT_FAILED");
    // Continue execution even if camera fails, system can still work partially
  }
  
  // Initialize slot status
  for (int i = 0; i < 2; i++) {
    slots[i].isBooked = false;
    slots[i].isOccupied = false;
    slots[i].bookedPlate = "";
  }
  
  // Request initial slot info from edge device
  Serial.println("REQUEST:SLOT_INFO");
  Serial.print("AVAILABLE_SLOTS:");
  Serial.println(availableSlots);
  
  // Send initial status to the edge device
  sendSystemStatus();
  
  Serial.println("SYSTEM:READY");
}

void loop() {
  // Check for commands from edge device
  checkSerialInput();
  
  // Update NTP time
  timeClient.update();
  
  // Check if gate should auto-close after timeout
  if (isGateOpen && (millis() - gateOpenTime > gateOpenDuration)) {
    closeGate();
  }
  
  // Request slot information from edge device periodically (every 5 seconds)
  static unsigned long lastDbUpdate = 0;
  if (millis() - lastDbUpdate > 5000) {
    Serial.println("REQUEST:SLOT_INFO");
    lastDbUpdate = millis();
  }
  
  // Send sensor data to the edge device periodically
  if (millis() - lastDataSentTime > DATA_SEND_INTERVAL) {
    // Format: SENSOR_DATA:GATE_IR:value,SLOT1_IR:value,SLOT2_IR:value
    Serial.print("SENSOR_DATA:GATE_IR:");
    Serial.print(digitalRead(gateIrSensorPin));
    Serial.print(",SLOT1_IR:");
    Serial.print(digitalRead(slot1IrSensorPin));
    Serial.print(",SLOT2_IR:");
    Serial.println(digitalRead(slot2IrSensorPin));
    
    // Update status of available slots
    Serial.print("AVAILABLE_SLOTS:");
    Serial.println(availableSlots);
    
    lastDataSentTime = millis();
  }
  
  // Check entrance sensor for vehicle detection
   if (digitalRead(gateIrSensorPin) == LOW && !isGateOpen) {
    Serial.println("VEHICLE:DETECTED_AT_ENTRANCE");
    Serial.println("Reading plate...");
    
    // Capture and recognize license plate
    detectedPlate = captureAndRecognizePlate();
    
    // Notify edge device about the detected plate for database storage
    if (detectedPlate.length() > 0 && detectedPlate != "NULL") {
      Serial.print("DETECTED_PLATE:");
      Serial.println(detectedPlate);
      
      // Find if this plate has a reserved slot - using our local data
      int matchingSlot = findMatchingSlot(detectedPlate);
      
      if (matchingSlot > 0) {
        // Car has a reservation
        Serial.println("Welcome! Go to Slot " + String(matchingSlot));
        Serial.print("SLOT_ASSIGNED:RESERVED:");
        Serial.println(matchingSlot);
        openGate();
        
        // Turn on green LED for the designated slot
        if (matchingSlot == 1) {
          digitalWrite(slot1RedLedPin, LOW);
          digitalWrite(slot1GreenLedPin, HIGH);
        } else {
          digitalWrite(slot2RedLedPin, LOW);
          digitalWrite(slot2GreenLedPin, HIGH);
        }
      } else {
        // No reservation found - check for available slots
        int availableSlot = findAvailableSlot();
        
        if (availableSlot > 0) {
          // Available slot found
          Serial.println("Available Slot: " + String(availableSlot));
          Serial.print("SLOT_ASSIGNED:AVAILABLE:");
          Serial.println(availableSlot);
          openGate();
          
          // Mark slot as occupied and update LEDs
          if (availableSlot == 1) {
            digitalWrite(slot1GreenLedPin, HIGH);
          } else {
            digitalWrite(slot2GreenLedPin, HIGH);
          }
        } else {
          // No available slots
          Serial.println("Sorry! No slots available");
          Serial.println("PARKING:FULL");
          activateBuzzer(500);
        }
      }
    } else {
      // Failed to detect plate
      Serial.println("Error: Plate not detected or invalid");
      Serial.println("PLATE:DETECTION_FAILED");
      activateBuzzer(500);
    }
  }
// Continuous wrong parking alert
for (int i = 0; i < 2; i++) {
  if (checkWrongParking(i)) {
    Serial.printf("Wrong Parking! Not your slot %d\n", i + 1);
    Serial.printf("WRONG_PARKING:SLOT%d\n", i + 1);
    
    // Loop beep while car is still wrongly parked
    while (checkWrongParking(i)) {
      activateBuzzer(200);  // Short beep
      delay(300);           // Pause between beeps
    }
  }
}
  
  // Update available slots count
  availableSlots = 0;
  for (int i = 0; i < 2; i++) {
    if (!slots[i].isBooked && !slots[i].isOccupied) {
      availableSlots++;
    }
  }
  
  // Check parking slots occupancy via IR sensors
  static bool lastSlot1Occupied = false;
  bool slot1Occupied = (digitalRead(slot1IrSensorPin) == LOW);
  slots[0].isOccupied = slot1Occupied;

  // Handle slot 1 status changes
  if (slot1Occupied != lastSlot1Occupied) {
    if (slot1Occupied) {
      //Car has arrived in slot 1
      Serial.println("SLOT1:OCCUPIED:" + detectedPlate);

      if (slots[0].isBooked) {
        // This car booked the slot → blink and keep LED on
        blinkLED(slot1GreenLedPin, 3, 300);
        digitalWrite(slot1GreenLedPin, HIGH);
      } else {
        // Unbooked car → just turn on LED
        blinkLED(slot1GreenLedPin, 3, 300);
        digitalWrite(slot1GreenLedPin, HIGH);
      }
    } else {
      // Car has left slot 1
      Serial.println("SLOT1:VACATED");
      digitalWrite(slot1GreenLedPin, LOW);
    }
    
    // Save new state
    lastSlot1Occupied = slot1Occupied;
    // updateSlotLEDs();
  }

  // Check parking slots occupancy via IR sensors
  static bool lastSlot2Occupied = false;
  bool slot2Occupied = (digitalRead(slot2IrSensorPin) == LOW);
  slots[1].isOccupied = slot2Occupied;

  // Handle slot 2 status changes
  if (slot2Occupied != lastSlot2Occupied) {
    if (slot2Occupied) {
      //Car has arrived in slot 2
      Serial.println("SLOT2:OCCUPIED:" + detectedPlate);

      if (slots[1].isBooked) {
        // This car booked the slot → blink and keep LED on
        blinkLED(slot2GreenLedPin, 3, 300);
        digitalWrite(slot2GreenLedPin, HIGH);
      } else {
        // Unbooked car → just turn on LED
        blinkLED(slot2GreenLedPin, 3, 300);
        digitalWrite(slot2GreenLedPin, HIGH);
      }
    } else {
      // Car has left slot 2
      Serial.println("SLOT2:VACATED");
      digitalWrite(slot2GreenLedPin, LOW);
    }
    
    // Save new state
    lastSlot2Occupied = slot2Occupied;
    // updateSlotLEDs();
  }
  
  // Output parking status through serial
  if (millis() - lastDbUpdate > 5000) {  // Update every 5 seconds
    String statusMessage = "Available: " + String(availableSlots) + " Slots: ";
    
    if (!slots[0].isBooked && !slots[0].isOccupied) {
      statusMessage += "1 ";
    }
    if (!slots[1].isBooked && !slots[1].isOccupied) {
      statusMessage += "2";
    }
    
    if (availableSlots == 0) {
      statusMessage += "No slots free";
    }
    
    Serial.println("STATUS_MESSAGE:" + statusMessage);
  }
  
  // Delay to prevent excessive looping
  delay(100);
}