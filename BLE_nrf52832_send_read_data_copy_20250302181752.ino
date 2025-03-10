/*********************************************************************
   Feather nRF52832 BLE Peripheral Example
   - Advertises a Custom Service with a single Notify Characteristic
   Written for Adafruit nRF52 Arduino Core.
   MIT License
*********************************************************************/

#include <bluefruit.h>
#include <Servo.h>

//--------------------------------------------------------------------
// Custom 128-bit Service & Characteristic UUIDs
//--------------------------------------------------------------------
#define SERVICE_UUID           "12345678-1234-5678-1234-56789abcdef0"
#define BATTERY_LIFE_UUID      "e540f441-a246-4072-b162-e2b67d4c7f57"
#define VENT_ANGLE_UUID        "abcdef01-1234-5678-1234-56789abcdef1"
// Create the BLE Service & Characteristic objects
BLEService        customService(SERVICE_UUID);
BLECharacteristic battery_life_char(BATTERY_LIFE_UUID);
BLECharacteristic vent_angle_char(VENT_ANGLE_UUID);

// Establish the servo motor
#define ServoPin 7
Servo myServo;

//--------------------------------------------------------------------
// Setup
//--------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  myServo.attach(ServoPin);
  pinMode(11, OUTPUT);

  Serial.println("nRF52832 BLE Peripheral");

  // 1) Initialize the Bluefruit stack
  Bluefruit.begin();
  Bluefruit.setName("nRF52832_Peripheral");  // Advertised device name
  Bluefruit.setTxPower(4);                  // Check local regulations

  // 2) Configure and start the custom service + characteristic
  setupServiceAndCharacteristic();

  // 3) Set up advertising
  startAdvertising();

  Serial.println("Setup complete, now advertising...");
}
float scaleFactor = 1.81 / 0.41;
// float scaleFactor = 1;
void send_battery_alert() {
      // Create a small string message
      char message[32];
      // snprintf(message, sizeof(message), "Hello #%lu", (unsigned long)counter++);
      // Send data for the battery consumption
      // float measuredvbat = analogRead(A0);
      // measuredvbat *= 2;    // we divided by 2, so multiply back
      // measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
      // measuredvbat /= 1024; // convert to voltage
      float measuredvbat; //= (float)analogRead(A1) * 3.3f / 4095.0f * scaleFactor;
      float total = 0;
      for (int i = 0; i < 10; i++) {
        total += (float)analogRead(A1) * 3.3f / 4095.0f * scaleFactor;
        delay(10);
      }
      measuredvbat = total / 10;
      measuredvbat = 5;
      Serial.print("Sent string: ");
      Serial.println(measuredvbat);
      
      snprintf(message, sizeof(message), "%0.3f", measuredvbat);
      
      // Notify new value
      bool success = battery_life_char.notify((uint8_t*)message, strlen(message));
      if (success) {
        Serial.print("battery: ");
        Serial.println(message);
      } else {
        Serial.println("Notification failed. Possibly not enabled?");
      }
}

void loop() {
  send_battery_alert();
  delay(1000);
  waitForEvent();
}

//--------------------------------------------------------------------
// Configure the custom service & characteristic
//--------------------------------------------------------------------
void setupServiceAndCharacteristic() {
  // 1) Initialize service
  customService.begin();

  // 2) Initialize characteristic
  //    - Set properties to Read + Notify
  //    - We set the max length to 20 bytes for demonstration
  battery_life_char.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  battery_life_char.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  battery_life_char.setMaxLen(20);     // up to 20 bytes (or more if using larger MTU)
  battery_life_char.begin();

  // Initialize the battery life characteristic
  //    - Set to write to the characteristic and notify other devices
  //    - Max length = 20 bytes
  vent_angle_char.setProperties(CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
  vent_angle_char.setPermission(SECMODE_OPEN, SECMODE_OPEN); // allows anybody to read/write to the characteristic
  vent_angle_char.setMaxLen(20);
  vent_angle_char.setWriteCallback(writeCallback);
  vent_angle_char.begin();

  // Optional: Set an initial value
  const char* initStr = "InitialValue";
  battery_life_char.write((uint8_t*)initStr, strlen(initStr));
}

//--------------------------------------------------------------------
// Advertising setup
//--------------------------------------------------------------------
void startAdvertising() {
  // Add flags, TX power, our custom service UUID, and the device name
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(customService);
  Bluefruit.Advertising.addName();

  // Configure advertising parameters
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // 20 ms - 152.5 ms
  Bluefruit.Advertising.setFastTimeout(30);   // 30 seconds of fast adv
  Bluefruit.Advertising.start(0);             // 0 = advertise forever
}

bool isNumeric (String input) {
  char c;
  if (input == "") {
    return false;
  }
  for (int i = 0; i < input.length(); i++) {
    c = input[i];
    if (c < '0' || c > '9') {
      return false;
    }
  }
  return true;
}

// Function that intakes an angle and writes it to the servo motor
void moveServo(int angle) {
  // Moves to the angle and waits for 5 seconds, then detaches to avoid consuming too much current
  Serial.print("Attempting to move to angle "); Serial.println(angle);
  myServo.attach(ServoPin);
  delay(100);
  myServo.write(angle);
  delay(1000); // will normally be 5 secs (5000), but changing to 1 sec (1000) for testing purposes
  myServo.detach();
}

// 
void moveServoFromString(String message) {
  if (message.equals("open")) {
    moveServo(165);
    digitalWrite(11, HIGH);
  }
  else if (message.equals("close")) {
    moveServo(20);
    digitalWrite(11, LOW);
  }
  else {
    Serial.println("DBG: In moveServoFromString: Invalid String");
  }

}

// functions that intakes data from the Vent Angle characteristic and adjusts the vent angle accordingly
void writeCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  String incomingMessage;
  incomingMessage = String((char*)data).substring(0, len); // Prevent buffer overflow

  Serial.print("[WriteChar] Received String: ");
  Serial.println(incomingMessage);

  if (isNumeric(incomingMessage)) {
    Serial.println("NUMBER DETECTED");
    moveServo(incomingMessage.toInt());
  }
  else {
    // Serial.println("INVALID INTEGER");
    // Assumes that the incoming message is either "open" or "close"
    moveServoFromString(incomingMessage);
  }
}
