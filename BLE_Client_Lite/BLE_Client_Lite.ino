/**
 * Example ESP32 BLE Client to connect to an nRF52832 (Adafruit) Peripheral
 * that advertises a custom Service (UUID: 4fafc201-1fb5-459e-8fcc-c5c9c331914b)
 * and has a characteristic (UUID: e540f441-a246-4072-b162-e2b67d4c7f57).
 *
 * Flow:
 *  1) Scan for BLE devices. 
 *  2) If a device advertises the target service UUID, stop scanning.
 *  3) Connect, discover the service and characteristic.
 *  4) Register for notifications (if available).
 *  5) Write data to the characteristic from the Serial Monitor commands.
 *
 * Adjust scanning intervals, power levels, or other parameters as needed.
 */

#include "BLEDevice.h"

// The remote service we wish to connect to.
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");

// The remote characteristic we are interested in (for vent angle).
static BLEUUID vent_angle_UUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

// The remote characteristic we are interested in (for battery life)
static BLEUUID battery_life_UUID("e540f441-a246-4072-b162-e2b67d4c7f57");

// The remote characteristic we are interested in (for battery life angle).

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;

static BLERemoteCharacteristic* pRemoteVentAngleCharacteristic   = nullptr;
static BLERemoteCharacteristic* pRemoteBatteryLifeCharacteristic = nullptr;

static BLEAdvertisedDevice*     myDevice  = nullptr;

// Callback invoked when a notification arrives from the peripheral
static void notifyCallback(
    BLERemoteCharacteristic* pBLERemoteCharacteristic,
    uint8_t* pData,
    size_t length,
    bool isNotify) 
{
  Serial.print("Notify callback for characteristic: ");
  Serial.println(pBLERemoteCharacteristic->getUUID().toString().c_str());

  Serial.print("Data length: ");
  Serial.println(length);

  Serial.print("Data: ");
  // If the peripheral is sending ASCII text, we can print as a string:
  // (Be careful if your data is raw binary, you may need hex printing instead)
  for (size_t i = 0; i < length; i++) {
    Serial.write(pData[i]);
  }
  Serial.println();
}

// Client (ESP32) connection callback
class MyClientCallback : public BLEClientCallbacks 
{
  void onConnect(BLEClient* pclient) {
    Serial.println("Connected to the peripheral.");
    connected = true;
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("Disconnected from the peripheral.");
  }
};

// Connect to the server (peripheral) that was found by onResult()
bool connectToServer() 
{
  if (!myDevice) {
    Serial.println("No device found to connect to!");
    return false;
  }

  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());
    
  BLEClient*  pClient  = BLEDevice::createClient();
  Serial.println(" - Created BLE client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remote BLE Server.
  pClient->connect(myDevice);  // This will use address type automatically
  Serial.println(" - Connected to server (attempted)");

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = nullptr;
  
  // Some boards require a short delay for service discovery to complete
  // You could do a small loop or repeated attempts:
  for (int i = 0; i < 10; i++) {
    pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService != nullptr) break;
    delay(100);
  }

  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our custom service");

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteVentAngleCharacteristic = pRemoteService->getCharacteristic(vent_angle_UUID);
  if (pRemoteVentAngleCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(vent_angle_UUID.toString().c_str());
    pClient->disconnect();
    return false;
  }

  // Obtain information for the battery life characteristic
  pRemoteBatteryLifeCharacteristic = pRemoteService->getCharacteristic(battery_life_UUID);
  if (pRemoteBatteryLifeCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(battery_life_UUID.toString().c_str());
    pClient->disconnect();
    return false;
  }

  Serial.println(" - Found our characteristic");

  // Optionally read the current value
  if (pRemoteVentAngleCharacteristic->canRead()) {
    String value = pRemoteVentAngleCharacteristic->readValue();
    Serial.print("Characteristic current value: ");
    Serial.println(value.c_str());
  }

  // If the characteristic supports notifications, register the callback
  if (pRemoteVentAngleCharacteristic->canNotify()) {
    pRemoteVentAngleCharacteristic->registerForNotify(notifyCallback);
    Serial.println(" - Notifications enabled (if CCCD is set on the peripheral).");
  }

  // Optionally read the current value
  if (pRemoteBatteryLifeCharacteristic->canRead()) {
    String value = pRemoteBatteryLifeCharacteristic->readValue();
    Serial.print("Characteristic current value: ");
    Serial.println(value.c_str());
  }

  // If the characteristic supports notifications, register the callback
  if (pRemoteBatteryLifeCharacteristic->canNotify()) {
    pRemoteBatteryLifeCharacteristic->registerForNotify(notifyCallback);
    Serial.println(" - Notifications enabled (if CCCD is set on the peripheral).");
  }
  
  connected = true;
  return true;
}

// Called for each device found during scanning
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks 
{
  void onResult(BLEAdvertisedDevice advertisedDevice) 
  {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // Check if the device is advertising our custom service
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      Serial.println(" - Found a device with our desired service UUID!");
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan    = true;
    }
  }
};

void setup() 
{
  Serial.begin(115200);
  Serial.println("Starting ESP32 BLE Client...");

  // Initialize BLE
  BLEDevice::init("");
  // Increase transmit power if desired:
  // BLEDevice::setPower(ESP_PWR_LVL_P9);

  // Retrieve a Scanner and set the callback
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);

  // Start scanning for up to 30 seconds (blocking mode = false).
  pBLEScan->start(30, false);
}

void loop() 
{
  // Once we've found a matching device, connect to it.
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("Failed to connect to the server; nothing more to do.");
    }
    doConnect = false;
  }

  // Example: read user input from Serial to send to the peripheral
  if (connected && Serial.available() > 0) {
    String str = Serial.readStringUntil('\n');
    str.trim(); // remove trailing newline etc.
    if (str.length() > 0) {
      Serial.print("Writing to peripheral: ");
      Serial.println(str);
      pRemoteVentAngleCharacteristic->writeValue(str.c_str(), str.length());
    }
  }


  
  // If we're disconnected but told to keep scanning (doScan),
  // automatically re-start the scan in case the peripheral shows up again
  if (!connected && doScan) {
    BLEDevice::getScan()->start(0);  // 0 = scan forever
  }

  delay(1000);
}
