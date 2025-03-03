/*********************************************************************
   Feather nRF52832 BLE Peripheral Example
   - Advertises a Custom Service with a single Notify Characteristic
   Written for Adafruit nRF52 Arduino Core.
   MIT License
*********************************************************************/

#include <bluefruit.h>

//--------------------------------------------------------------------
// Custom 128-bit Service & Characteristic UUIDs
//--------------------------------------------------------------------
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BATTERY_LIFE_UUID      "e540f441-a246-4072-b162-e2b67d4c7f57"
#define VENT_ANGLE_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26a8"
// Create the BLE Service & Characteristic objects
BLEService customService(SERVICE_UUID);
BLECharacteristic battery_life_char(BATTERY_LIFE_UUID);
BLECharacteristic vent_angle_char(VENT_ANGLE_UUID);

//--------------------------------------------------------------------
// Setup
//--------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("nRF52832 BLE Peripheral Demo");

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

void loop() {
  static uint32_t counter = 0;
  static uint32_t lastMillis = 0;

  // Once connected and if the central enabled notifications,
  // we can periodically send an update.
  if (Bluefruit.connected()) {
    uint32_t now = millis();
    if (now - lastMillis >= 1000) {
      lastMillis = now;

      // Create a small string message
      char message[32];
      // snprintf(message, sizeof(message), "Hello #%lu", (unsigned long)counter++);
      // Send data for the battery consumption
      float measuredvbat = analogRead(A7);
      measuredvbat *= 2;    // we divided by 2, so multiply back
      measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
      measuredvbat /= 1024; // convert to voltage
      snprintf(message, sizeof(message), "Battery = %0.2f", measuredvbat);
      
      // Notify new value
      bool success = battery_life_char.notify((uint8_t*)message, strlen(message));
      if (success) {
        Serial.print("Notified: ");
        Serial.println(message);
      } else {
        Serial.println("Notification failed. Possibly not enabled?");
      }
    }
  }
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


void writeCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  Serial.print("[WriteChar] Data received: ");
  String incomingMessage;
  incomingMessage = String((char*)data).substring(0, len); // Prevent buffer overflow

  Serial.print("[WriteChar] Received String: ");
  Serial.println(incomingMessage);
}
