/************************************************************
*
*   B.R.E.A.T.H.E Control and Sensor Sub System Code
*   Written By Patrick, assisted by Marcus, Emily, and Kyle
*T
************************************************************/

//original spot 
//revert back to here

#include <SPI.h>                          //include necessary libraries such as spi, i2c, and tft
#include <TFT_eSPI.h>           
#include <Wire.h>
#include <SensirionI2CScd4x.h>
#include "inter.h"                        //define font library
//#include "interlarge.h"
#include "med.h"
//#include "large.h"
#include "small.h"

#include "DHT20.h"

//new beginning

//1 - include BLE libraries
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define TFT_BL 17                        //define backlight pin  
#define BUZZER 33                          //define buzzer pin                
#define TP_BUSY 4                         //changed from 2 to 4
//#define TP_CS 13
#define TP_IRQ 13                          //changed from 12 to 13
#define SD_CS 2

#define CALIBRATION_FILE "/TouchCalData3" //for calibration files, not used
#define REPEAT_CAL false
//breakpoint 1
#define LEDC_TIMER_12_BIT 12

// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ 5000

#define BL_FREQ 10000

// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define LED_PIN 25

//for low battery sound
#define ONVALUE 200
#define quarter_note_length 200
#define NOTE_Ab NOTE_Gs



//begin defining frame sizes 
// Switch position and size
#define FRAME_X 10 //330 for right, 10 for left
#define FRAME_Y 82
#define FRAME_W 135
#define FRAME_H 50

// Red zone size
#define REDBUTTON_X FRAME_X
#define REDBUTTON_Y FRAME_Y
#define REDBUTTON_W (FRAME_W / 2)
#define REDBUTTON_H FRAME_H

// Green zone size
#define GREENBUTTON_X (REDBUTTON_X + REDBUTTON_W)
#define GREENBUTTON_Y FRAME_Y
#define GREENBUTTON_W (FRAME_W / 2)
#define GREENBUTTON_H FRAME_H

// Test button size

#define TESTBUTTON_X FRAME_X
#define TESTBUTTON_Y (REDBUTTON_Y + FRAME_H + 10)  // Positioned below the mode buttons
#define TESTBUTTON_W FRAME_W
#define TESTBUTTON_H 40

// Set button position and size
#define SETBUTTON_X FRAME_X
#define SETBUTTON_Y (GREENBUTTON_Y + GREENBUTTON_H + 10)  // Below the mode buttons
#define SETBUTTON_W FRAME_W
#define SETBUTTON_H 40


//test 
unsigned long lastBatteryCharmTime = 0;
const unsigned long batteryCharmInterval = 500;  // Adjust the interval as needed


// Move plus/minus buttons to the right side
//#define MINUSBUTTON_X 400  // Adjusted X position for right side
//#define MINUSBUTTON_Y (TESTBUTTON_Y + TESTBUTTON_H + 10)

int MINUSBUTTON_Y = TESTBUTTON_Y + TESTBUTTON_H + 10;
int PLUSBUTTON_Y = MINUSBUTTON_Y - 145; // Move up by 70 pixels

const int MINUSBUTTON_X_ORIG = 400;
const int PLUSBUTTON_X_ORIG = MINUSBUTTON_X_ORIG;

// Use variables that will be modified dynamically
int MINUSBUTTON_X = MINUSBUTTON_X_ORIG;
int PLUSBUTTON_X = PLUSBUTTON_X_ORIG;




//#define PLUSBUTTON_X (MINUSBUTTON_X)  // Place + button next to -
//#define PLUSBUTTON_Y (MINUSBUTTON_Y - 145)  // Move up by 70 pixels
#define BUTTON_WIDTH 50
#define BUTTON_HEIGHT 40

// Set button position and size
#define SETBUTTON_X FRAME_X
#define SETBUTTON_Y (REDBUTTON_Y + FRAME_H + 10)  // Below the Test button
#define SETBUTTON_W FRAME_W
#define SETBUTTON_H 40

TFT_eSPI tft = TFT_eSPI();       // Invoke custom library
TFT_eSprite sprite = TFT_eSprite(&tft);  // Create sprite object for text
//TFT_eSprite sprite = TFT_eSprite(&tft);  // Create sprite object for text
//TFT_eSprite tempSprite = TFT_eSprite(&tft);  // Separate from the shared "sprite"
//TFT_eSprite co2Sprite = TFT_eSprite(&tft);
SensirionI2CScd4x scd4x;         // Create SCD4x sensor object
DHT20 DHT;

int currentValue = 70; // Initial value


unsigned long lastUpdateTime = 0; // Store the last update time
const unsigned long updateInterval = 2000; // Update every 2 seconds

bool testButtonState = false; // To track the state of the test button (pressed or not)
bool SwitchOn = false;  // Variable to track if in manual mode

// Define green square position
int squareX = 240;  // Center X position
int squareY = 160;  // Center Y position
const int squareSize = 30; // Square size

int valueX = PLUSBUTTON_X - 42 + (FRAME_W / 2);  // Initial X position
int valueY = PLUSBUTTON_Y + BUTTON_HEIGHT + 50;  // Keep the same Y position

//song
int song_pt;

uint8_t connection_status = 100;

String ventStatus = "OPEN";  // Default status is OPEN

#define VENT_SERVICE_UUID         "12345678-1234-5678-1234-56789abcdef0"  // Replace with actual UUID
#define VENT_CHARACTERISTIC_UUID  "12344209-1234-5678-1234-56789abcdef0"  // Replace with actual UUID
#define BATTERY_LIFE_CHARACTERISTIC_UUID      "12349823-1234-5678-1234-56789abcdef0"
static BLEUUID serviceUUID(VENT_SERVICE_UUID);
static BLEUUID characteristicUUID(VENT_CHARACTERISTIC_UUID);
static BLEUUID battery_life_UUID(BATTERY_LIFE_CHARACTERISTIC_UUID);

static boolean doConnect = false;
static boolean isConnected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* ventCharacteristic = nullptr;
static BLERemoteCharacteristic* batteryCharacteristic = nullptr;
static BLEAdvertisedDevice* myDevice = nullptr;

//breakpoint for testing co2 hysteresis 
uint16_t co2Readings[5];
bool triggerCO2 = false;
bool actionTriggered = false; 
int co2Index = 0;
uint16_t co2 = 0;
float temperatureF = 0.0; //temp

String BatteryLife = "";

//3/28/25 breakpoint(1)
//Battery Life Variables
const float LOW_BATTERY_THRESHOLD = 1.4;               //Set the voltage threshold for low battery to 1.2V
const float DEAD_BATTERY_THRESHOLD = 1.1; // Threshold where it is considered dead

const int READING_COUNT = 3;                         //Set the size of the array storing voltage values
const int MIN_READING_FOR_AVERAGE = 2;                //Set the minimum readings needed before calculating average to 3
const unsigned long VERIFICATION_DELAY_MS = 5000;     //Set the verificaiton period to 5 seconds 

static bool batteryLow = false;                       //Initially set the low battery status to false
static bool batteryDead=false;

static bool batteryLowPending = false;                //Initially set the pending low battery status to false 
static unsigned long lowBatteryDetectionTime = 0;     //Time when the low battery was first detected

float voltageReadings[READING_COUNT];                 //Array storing the voltage  readings
int currentReadingIndex = 0;                          //Current position in the array
bool bufferFilled = false;                            //Flag indicating array is full of the past 10 values

//float averageVoltage = 0.0;                         //disable this during actual testing
float averageVoltage;                             //disable this during prototyping
//end of breakpoint(1)


//breakpoint (3)
bool newDataReceived = false;  // Flag to check if new data has been received


//bool for snooze
bool snoozeActive = false;
unsigned long snoozeTime = 0;
const unsigned long snoozeDuration = 5000;



TaskHandle_t BuzzerTaskHandle = NULL; 

bool isOpen = false;


#define SNOOZEBUTTON_X 270  // Position X of the snooze button (matches batteryStatusBox)
#define SNOOZEBUTTON_Y 275  // Position Y of the snooze button (matches batteryStatusBox)
#define SNOOZEBUTTON_W 200  // Width of the snooze button (adjust as needed)
#define SNOOZEBUTTON_H 50   // Height of the snooze button (adjust as needed)




void drawSnoozeButton() {
  // Draw the snooze button at the given position and size
  sprite.deleteSprite();  // Clear previous sprite before creating a new one
  sprite.setColorDepth(8);
  sprite.createSprite(SNOOZEBUTTON_W, SNOOZEBUTTON_H);  // Create sprite matching the previous text area
  sprite.fillSprite(0x4208);  // Fill background with original rectangle color

  sprite.loadFont(inter);  // Load Inter font
  sprite.setTextColor(TFT_WHITE);  // Set text color
  sprite.setTextDatum(MC_DATUM);   // Center text inside sprite

    // Draw vent status inside the sprite
  sprite.drawString("Snooze", SNOOZEBUTTON_W / 2, SNOOZEBUTTON_H /2);  

    // Push sprite to screen at the designated position
  sprite.pushSprite(SNOOZEBUTTON_X, SNOOZEBUTTON_Y);
  sprite.unloadFont(); // Unload font to free memory
}

static void notifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    // Update BatteryLife with the received data
    BatteryLife = String((char*)pData).substring(0, length);
    Serial.print("Notification received: ");
    Serial.println(BatteryLife);

    if(pChar->getUUID().toString().equals(BATTERY_LIFE_CHARACTERISTIC_UUID)){
      newDataReceived = true;  // Mark new data as received
    }

}

void lowBatteryCharmLoop() {

  while (batteryLow == true){
    lowBatteryCharm();  // Play the low battery sound
        delay(500);  // Add a small delay to avoid overwhelming the system, adjust if needed

  }
}

void lame(){
    if (!newDataReceived) {
        return;  // If no new data, do nothing
    }

    // Convert BatteryLife to float
    float voltage = BatteryLife.toFloat();
    
    // Only process valid numbers (not NAN)
    if (!isnan(voltage) && connection_status != 0) {
        // Print the new voltage reading
        Serial.print("New voltage reading: ");
        Serial.print(voltage);
        Serial.println("V");

        // Store reading in circular buffer
        voltageReadings[currentReadingIndex] = voltage;
        currentReadingIndex = (currentReadingIndex + 1) % READING_COUNT;  // Wrap around

        // Mark buffer as filled after first complete cycle
        if (!bufferFilled && currentReadingIndex == 0) {
            bufferFilled = true;
        }

        // Print current buffer contents
        Serial.print("Current Battery Readings: [");
        int count = bufferFilled ? READING_COUNT : currentReadingIndex;  // Number of valid readings
        for (int i = 0; i < count; i++) {
            // Calculate proper index for circular buffer
            int idx = (currentReadingIndex + i) % (bufferFilled ? READING_COUNT : currentReadingIndex);
            if (i > 0) Serial.print(", ");  // Add comma separator
            Serial.print(voltageReadings[idx]);  // Print reading
        }
        Serial.println("]");

        // Only calculate average if we have enough readings
        if (count >= MIN_READING_FOR_AVERAGE) {
            float sum = 0;
            int validCount = 0;

            // Sum all valid readings
            for (int i = 0; i < count; i++) {
                int idx = (currentReadingIndex + i) % (bufferFilled ? READING_COUNT : currentReadingIndex);
                if (!isnan(voltageReadings[idx])) {
                    sum += voltageReadings[idx];
                    validCount++;
                }
            }

            // Calculate and display average if we have valid readings
            if (validCount > 0) {
                averageVoltage = sum / validCount;
                Serial.print("Average (");
                Serial.print(validCount);
                Serial.print(" readings): ");
                Serial.print(averageVoltage);
                Serial.println("V");


                // Battery status verification logic (without timer)
                if (averageVoltage < LOW_BATTERY_THRESHOLD && averageVoltage > 1.2) {
                    if (!batteryLowPending && !batteryLow) {
                        // First detection of low voltage
                        batteryDead = false;
                        batteryLowPending = true;
                        lowBatteryDetectionTime = millis();  // Record detection time
                        Serial.println("Potential low battery, timer will now verify");
                        Serial.println("voltage is less than 1.4 but greater than 1.2");
                        batteryStatusBox(270, 275, "Ptnl. Low Battery", TFT_ORANGE);
                    } 
                    else if (batteryLowPending && (millis() - lowBatteryDetectionTime >= VERIFICATION_DELAY_MS)) {
                        // Low battery confirmed after verification period
                        batteryDead = false;
                        batteryLowPending = false;
                        batteryLow = true;
                        Serial.println("CONFIRMED: LOW BATTERY!");
                        Serial.println("Buzzer activated");
                        batteryStatusBox(8, 275, "Low Battery!", TFT_ORANGE);
                        batteryStatusBox(270, 275, " ",TFT_ORANGE);
                        //drawSnoozeButton();
                        lowBatteryCharm();
                        

                        
                        
                        
                        //lowBatteryCharmLoop();
                        
                    }
                } 

                
                else if (averageVoltage > LOW_BATTERY_THRESHOLD){
                    Serial.println("Above 1.4");
                    //drawStatusBox(270, 275, " ", 0x18c3);
                    emptyBatteryMsg();  // Clear battery box
                    batteryLow = false;
                    batteryDead = false;
                    ledcWrite(BUZZER, 0);
                    // Battery voltage is normal
                    if (batteryLowPending) {
                        // Recovered during verification period
                        batteryLowPending = false;
                        Serial.println("False alarm, battery recovered during verification");
                        emptyBatteryMsg();  // Clear battery box
                        ledcWrite(BUZZER, 0);

                    }
                    else if (batteryLow) {
                        // Recovered after being confirmed low
                        batteryLow = false;
                        Serial.println("Battery back to normal");
                        Serial.println("Buzzer Off");
                        batteryStatusBox(270, 275, " ", 0x18c3);
                        emptyBatteryMsg();  // Clear battery box
                        ledcWrite(BUZZER, 0);

                    }
                    
                }

                  else if(averageVoltage < 1.2) {
                  batteryDead = true;
                  Serial.println("Dead, Below 1.2");
                  batteryStatusBox(8, 275, "   Vent Out of Battery!", TFT_ORANGE);
                  batteryStatusBox(270, 275, " ", TFT_ORANGE);

                  
                }

 
            }
        } else {
            Serial.println("Not enough readings for average yet");
        }
    } else {
        Serial.println("Received invalid voltage data!");
    }

    newDataReceived = false;  // Reset the flag after processing
}







class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) override {
        Serial.println("BLE Client Connected!");
    }

    void onDisconnect(BLEClient* pclient) override {
        Serial.println("BLE Client Disconnected! Restarting scan...");
        isConnected = false;
    }
};

bool connectToServer() {
    if (myDevice == nullptr) {
        Serial.println(" No device found to connect to.");
        return false;
    }

    Serial.print("Connecting to Vent at: ");
    Serial.println(myDevice->getAddress().toString().c_str());

    BLEClient* pClient = BLEDevice::createClient();
    Serial.println(" - Created BLE client");
    pClient->setClientCallbacks(new MyClientCallback());

    if (!pClient->connect(myDevice)) {
        Serial.println(" BLE Connection Failed!");
        isConnected = false;
        return false;
    }

    Serial.println(" Connected to Vent!");
    pClient->setMTU(517);  // Set max MTU for better data transfer

    //  Get the service
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
        Serial.println(" Failed to find Vent Service UUID!");
        pClient->disconnect();
        return false;
    }

    Serial.println(" - Found Vent service!");

    // Get the characteristic
    ventCharacteristic = pRemoteService->getCharacteristic(characteristicUUID);
    if (ventCharacteristic == nullptr) {
        Serial.println(" Failed to find Vent Characteristic UUID!");
        pClient->disconnect();
        return false;
    }

    Serial.println(" - Found Vent characteristic!");

    // Grab the Battery Life Characteristic
    batteryCharacteristic = pRemoteService->getCharacteristic(battery_life_UUID);
    if (batteryCharacteristic == nullptr) {
        Serial.println(" Failed to find Vent Characteristic UUID!");
        pClient->disconnect();
        return false;
    }

    // Enable notifications if available
    if (batteryCharacteristic->canNotify()) {
        batteryCharacteristic->registerForNotify(notifyCallback);
    }

  if (ventCharacteristic->canNotify()) {
    ventCharacteristic->registerForNotify(notifyCallback);
   
  }


    isConnected = true;
    return true;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        Serial.print(" BLE Device Found: ");
        Serial.println(advertisedDevice.toString().c_str());

        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
            BLEDevice::getScan()->stop();
            myDevice = new BLEAdvertisedDevice(advertisedDevice);
            doConnect = true;
            doScan = true;
        }
    }
};

void initializeBLEScan() {
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
}

void startBLEScan() {
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->start(10, false);
}

void sendVentCommand(String command) {
    Serial.print(" Checking BLE connection... isConnected: ");
    Serial.println(isConnected);

    if (isConnected && ventCharacteristic != nullptr) {
        ventCharacteristic->writeValue(command.c_str(), command.length());
        Serial.print("BLE Message Sent: ");
        Serial.println(command);
    } else {
        Serial.println("Not connected to Vent! Retrying connection...");
        connectToServer();
    }
}

void sendTemperatureValue() {
    if (isConnected && ventCharacteristic != nullptr) {
        String tempCommand = "set:" + String(currentValue);  // Format message
        ventCharacteristic->writeValue(tempCommand.c_str(), tempCommand.length());
        Serial.print("BLE Message Sent: ");
        Serial.println(tempCommand);
    } else {
        Serial.println("Not connected to Vent! Retrying connection...");
        connectToServer();
    }
}

// Call these inside your button press functions
void openVent() {
    sendVentCommand("OPEN\0");
    ventStatus = "OPEN";
    isOpen = true;
}

void closeVent() {
    sendVentCommand("CLOSE\0");
    ventStatus = "CLOSE";
    isOpen = false;
}

void emptyBatteryMsg(){
  // params: x, y, w, h, color
  tft.fillRect(0, 275, 480, 60, 0x18c3);
}

//breakpoint 1000

void ledcAnalogWrite(uint8_t pin, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 4095 from 2 ^ 12 - 1
  uint32_t duty = (4095 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWrite(pin, duty);
}

void snoozeBuzzer(){
    ledcAnalogWrite(BUZZER, 0);  // Turn off the buzzer
    snoozeActive = true;
}
//for low battery sound:
//-------------------------------------------------------------------//
void restNote(int time) {
  analogWrite(BUZZER, 0);
  delay(time);
  //analogWrite(BUZZER, ONVALUE);
}

void playNote(note_t note, int timeLength, int octave) {
  // play the note for 90% of the time then turn off for a little
  analogWrite(BUZZER, ONVALUE);
  ledcWriteNote(BUZZER, note, octave);
  delay(timeLength * 0.9);
  restNote(timeLength * 0.1);
}

void playNote_legato(note_t note, int timeLength, int octave) {
  ledcWriteNote(BUZZER, note, octave);
  delay(timeLength);
}


void lowBatteryCharm() {
  uint16_t x, y;
  // measure 1/2
  for (int i = 0; i < 3; i++) {
    playNote(NOTE_G, quarter_note_length / 2, 4);
  }
  playNote(NOTE_Eb, quarter_note_length * 3, 4);
  // measure 3-5
  restNote(quarter_note_length / 2);
  for (int i = 0; i < 3; i++) {
    playNote(NOTE_F, quarter_note_length / 2, 4);
  }
  playNote(NOTE_D, quarter_note_length * 3, 4);

  // measure 6
  restNote(quarter_note_length / 2);
  for (int i = 0; i < 3; i++) {
    playNote(NOTE_G, quarter_note_length / 2, 4);
  }
  // measure 7
  playNote(NOTE_Eb, quarter_note_length / 2, 4);
  for (int i = 0; i < 3; i++) {
    playNote(NOTE_Ab, quarter_note_length / 2, 4);
  }
  //measure 8
  playNote(NOTE_G, quarter_note_length / 2, 4);
  for (int i = 0; i < 3; i++) {
    playNote(NOTE_Eb, quarter_note_length / 2, 5);
  }
  // measure 9
  playNote_legato(NOTE_C, quarter_note_length * 2, 5);
  //measure 10
  playNote(NOTE_C, quarter_note_length / 2, 5);
  for (int i = 0; i < 3; i++) {
    playNote(NOTE_G, quarter_note_length / 2, 4);
  }

  //measure 11
  playNote(NOTE_D, quarter_note_length / 2, 4);
  for (int i = 0; i < 3; i++) {
    playNote(NOTE_Ab, quarter_note_length / 2, 4);
  }

  //measure 12
  playNote(NOTE_G, quarter_note_length / 2, 4);
  for (int i = 0; i < 3; i++) {
    playNote(NOTE_F, quarter_note_length / 2, 5);
  }

  //measure 13
  playNote(NOTE_D, quarter_note_length * 2, 5);
  //measure 14
  playNote(NOTE_D, quarter_note_length / 2, 5);
  for (int i = 0; i < 2; i++) {
    playNote(NOTE_G, quarter_note_length / 2, 5);
  }
  playNote(NOTE_F, quarter_note_length / 2, 5);

  //measure 15
  playNote(NOTE_Eb, quarter_note_length * 2, 5);

  //measure 16
  playNote(NOTE_D, quarter_note_length / 2, 5);
  for (int i = 0; i < 2; i++) {
    playNote(NOTE_G, quarter_note_length / 2, 5);
  }
  playNote(NOTE_F, quarter_note_length / 2, 5);

  //measure 17
  playNote(NOTE_Eb, quarter_note_length * 2, 5);

  //measure 18
  playNote(NOTE_D, quarter_note_length / 2, 5);
  for (int i = 0; i < 2; i++) {
    playNote(NOTE_G, quarter_note_length / 2, 5);
  }
  playNote(NOTE_F, quarter_note_length / 2, 5);

  //measure 19
  playNote(NOTE_Eb, quarter_note_length, 5);
  restNote(quarter_note_length);

  //measure 20
  playNote(NOTE_C, quarter_note_length, 5);
  restNote(quarter_note_length);

  //measure 21
  playNote(NOTE_G, quarter_note_length * 3, 5);

  //measure 22
  restNote(quarter_note_length / 2);
  for (int i = 0; i < 3; i++) {
    playNote(NOTE_Ab, quarter_note_length / 2, 5);
  }

  //measure 23
  playNote(NOTE_F, quarter_note_length * 6, 5);

  restNote(50);

  if (tft.getTouch(&x, &y)){
    return;
  }
}

//-------------------------------------------------------------------//


void update_connect_status (String message) {
    // 1) Clear that region on the TFT
    tft.fillRect(177, 0, 177, 40, TFT_BLACK);

    // 2) Set text properties (same as in the sprite)
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);

    // 3) Draw your string at the coordinates 
    //    that replicate sprite offset (80,24)
    tft.drawString(message.c_str(), 177 + 80, 0 + 24); 
}

uint16_t BLE_counter = 0;
#define max_BLE_counter 40
void handleBluetooth() {
    static bool wasConnected = false; // Tracks previous connection status
    Serial.print("BLE_counter = "); Serial.println(BLE_counter);

    if (!isConnected && BLE_counter == max_BLE_counter) {
      // print "connecting..."
      if (connection_status != 1) {
        update_connect_status ("Connecting...");
        connection_status = 1;
      }
      startBLEScan();
      BLE_counter = 0;
    }
    else if (BLE_counter != max_BLE_counter) {
      BLE_counter++;
    }
    // if it is not connected, then just print ("not connected")
    if (!isConnected) {
      // print not connected
      if (connection_status != 0 ) {
        update_connect_status("Not connected");
        for (int i = 0; i < READING_COUNT; i++){
          voltageReadings[i] = 1.5;
        }
        batteryStatusBox(270, 275, " ", 0x18c3);
        emptyBatteryMsg();  // Clear battery box
        ledcWrite(BUZZER, 0);
        connection_status = 0;
      }
    }
    else if (isConnected) {
      // print "connected"
      if (connection_status != 2) {
        update_connect_status("Connected");
        connection_status = 2;
      }
    }
    if (doConnect) {
        if (connectToServer()) {
            Serial.println("Connected to Vent successfully.");
            wasConnected = true;
        } else {
            Serial.println("Connection to Vent failed.");
            wasConnected = false;
        }
        doConnect = false;
    }

    if (isConnected) {
        if (!wasConnected) { // Only print once when connection is first established
            Serial.println("Vent is already connected.");
            wasConnected = true; // Update status so it doesn't print again
        }
    } else if (doScan) {
        Serial.println("Restarting BLE scan...");
        wasConnected = false; // Update status so it prints again when reconnected
    }

    // Print connection status every time it's checked
    // if (isConnected) {
    //     Serial.println("Connected");
    // } else {
    //     Serial.println("Not connected");
    // }
}





void drawVentStatus() {
    sprite.deleteSprite();  // Clear previous sprite before creating a new one

    sprite.createSprite(SETBUTTON_W, 30);  // Create sprite matching the previous text area
    sprite.fillSprite(0x1082);  // Fill background with original rectangle color

    sprite.loadFont(inter);  // Load Inter font
    sprite.setTextColor(TFT_WHITE);  // Set text color
    sprite.setTextDatum(MC_DATUM);   // Center text inside sprite

    // Draw vent status inside the sprite
    sprite.drawString(ventStatus, SETBUTTON_W / 2, 15);  

    // Push sprite to screen at the designated position
    sprite.pushSprite(SETBUTTON_X, SETBUTTON_Y + SETBUTTON_H + 25);

    sprite.unloadFont(); // Unload font to free memory
}


void drawGreenSquare() {
  tft.fillRect(squareX, squareY, squareSize, squareSize, TFT_GREEN);
}

void drawPlusMinusButtons() {
   int radius = 28;  // Radius of the buttons (adjust as needed)

   // Draw minus button as a circle
   tft.fillCircle(MINUSBUTTON_X + radius, MINUSBUTTON_Y + radius, radius, 0x18e3);
   tft.setTextColor(TFT_WHITE);
   tft.setTextSize(2);
   tft.setTextDatum(MC_DATUM);
   tft.drawString("-", MINUSBUTTON_X + radius, MINUSBUTTON_Y + radius);

   // Draw plus button as a circle
   tft.fillCircle(PLUSBUTTON_X + radius, PLUSBUTTON_Y + radius, radius, 0x18e3);
   tft.setTextColor(TFT_WHITE);
   tft.setTextSize(2);
   tft.setTextDatum(MC_DATUM);
   tft.drawString("+", PLUSBUTTON_X + radius, PLUSBUTTON_Y + radius);
}

const int OPENBUTTON_X_ORIG = 800;  // Move off-screen initially
const int CLOSEBUTTON_X_ORIG = 800;

const int OPENBUTTON_X_VISIBLE = 340;  // Manual Mode position
const int CLOSEBUTTON_X_VISIBLE = 340;

const int OPENBUTTON_Y_ORIG = 80;
const int CLOSEBUTTON_Y_ORIG = 180;
const int BUTTON_W = 130;
const int BUTTON_H = 60;

// Use dynamic variables that will be modified
int OPENBUTTON_X = OPENBUTTON_X_ORIG;
int CLOSEBUTTON_X = CLOSEBUTTON_X_ORIG;
int OPENBUTTON_Y = OPENBUTTON_Y_ORIG;
int CLOSEBUTTON_Y = CLOSEBUTTON_Y_ORIG;

void drawSetButton() {
    sprite.deleteSprite();  // Clear previous sprite
    sprite.setColorDepth(8);
    sprite.createSprite(SETBUTTON_W, SETBUTTON_H);  // Create sprite for button area

    if (SwitchOn) {  // Autonomous Mode
        sprite.fillSprite(0x4208);  // Green background for Autonomous mode

        sprite.loadFont(small);  
        sprite.setTextColor(TFT_WHITE, 0x4208);  // White text for contrast
        sprite.setTextDatum(MC_DATUM);
        sprite.drawString("Status", SETBUTTON_W / 2, SETBUTTON_H / 2);
        sprite.unloadFont();
    } 
    
    else {  // Manual Mode (Disabled)
        sprite.fillSprite(0x4208);  // Green background for Autonomous mode

        sprite.loadFont(small);  
        sprite.setTextColor(TFT_WHITE, 0x4208);  // White text for contrast
        sprite.setTextDatum(MC_DATUM);
        sprite.drawString("Status", SETBUTTON_W / 2, SETBUTTON_H / 2);
        sprite.unloadFont();
    }
    

    sprite.pushSprite(SETBUTTON_X, SETBUTTON_Y);  // Draw sprite to screen
}





void drawOpenCloseButtons() {
    // Draw Open button
    tft.fillRoundRect(OPENBUTTON_X, OPENBUTTON_Y, BUTTON_W, BUTTON_H, 10, 0x18e3);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Open", OPENBUTTON_X + (BUTTON_W / 2), OPENBUTTON_Y + (BUTTON_H / 2));

    // Draw Close button
    tft.fillRoundRect(CLOSEBUTTON_X, CLOSEBUTTON_Y, BUTTON_W, BUTTON_H, 10, 0x18e3);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Close", CLOSEBUTTON_X + (BUTTON_W / 2), CLOSEBUTTON_Y + (BUTTON_H / 2));
}

void updateOC(int deltaX) {
    // Clear previous button positions
    tft.fillRoundRect(OPENBUTTON_X, OPENBUTTON_Y, BUTTON_W, BUTTON_H, 10, 0x0841);  // Erase Open button
    tft.fillRoundRect(CLOSEBUTTON_X, CLOSEBUTTON_Y, BUTTON_W, BUTTON_H, 10, 0x0841); // Erase Close button

    // Move buttons relative to their original positions
    OPENBUTTON_X = OPENBUTTON_X_ORIG + deltaX;
    CLOSEBUTTON_X = CLOSEBUTTON_X_ORIG + deltaX;

    // Redraw buttons in new position
    drawOpenCloseButtons();
        Serial.print("Updated OPENBUTTON_X: ");
    Serial.println(OPENBUTTON_X);
    Serial.print("Updated CLOSEBUTTON_X: ");
    Serial.println(CLOSEBUTTON_X);
}




void updateButtons(int deltaX) {
    // Clear previous button positions by drawing over them with background color
    tft.fillCircle(MINUSBUTTON_X + 28, MINUSBUTTON_Y + 28, 28, 0x0841); // Background color
    tft.fillCircle(PLUSBUTTON_X + 28, PLUSBUTTON_Y + 28, 28, 0x0841); // Background color

    // Move buttons relative to their original positions
    MINUSBUTTON_X = MINUSBUTTON_X_ORIG + deltaX;
    PLUSBUTTON_X = PLUSBUTTON_X_ORIG + deltaX;

    // Redraw buttons in new position
    drawPlusMinusButtons();
}
// Function to update the green square position
void updateGreenSquare(int deltaY) {
  // Clear the previous square by drawing over it with the background color
  tft.fillRect(squareX, squareY, squareSize, squareSize, 0x0841);
  
  // Update position
  squareY += deltaY;
  

  
  // Redraw the square in the new position
  drawGreenSquare();
}


void batteryStatusBox(int x, int y, String text, uint16_t color){
  sprite.deleteSprite();
  sprite.setColorDepth(8);

  sprite.createSprite(300, 50);
  sprite.fillSprite(color);
  sprite.loadFont(small);
  sprite.setTextColor(TFT_WHITE);
  sprite.setTextDatum(MC_DATUM);
  sprite.drawString(text, 106, 25);
  sprite.pushSprite(x,y);
  sprite.unloadFont();

}

// Function to draw a standard status box
void drawStatusBox(int x, int y, String text, uint16_t color) {
    sprite.deleteSprite();  // Clear any previous sprite before creating a new one
    sprite.setColorDepth(8);

    sprite.createSprite(300, 50);  // Create sprite with standard width and height
    sprite.fillSprite(color);  // Fill the sprite background with the given color

    sprite.loadFont(inter);  // Load Inter font
    sprite.setTextColor(TFT_WHITE);  // Set text color
    sprite.setTextDatum(MC_DATUM);   // Center text inside the sprite

    sprite.drawString(text, 120, 25);  // Draw text in the center of the sprite

    sprite.pushSprite(x, y);  // Push sprite to TFT at specified position

    sprite.unloadFont(); // Unload font to free memory
}





void drawFrame()
{
  tft.drawRect(FRAME_X, FRAME_Y, FRAME_W, FRAME_H, TFT_BLACK);
}



// Draw a red button (Manual Mode)
void redBtn()
{
  tft.fillRect(REDBUTTON_X, REDBUTTON_Y, REDBUTTON_W, REDBUTTON_H, 0xd861);
  tft.fillRect(GREENBUTTON_X, GREENBUTTON_Y, GREENBUTTON_W, GREENBUTTON_H, TFT_DARKGREY);
  drawFrame();
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("A", GREENBUTTON_X + (GREENBUTTON_W / 2), GREENBUTTON_Y + (GREENBUTTON_H / 2));
  SwitchOn = false;

  // Draw the box with text above the button for manual mode
  tft.fillRect(REDBUTTON_X, REDBUTTON_Y - 30, FRAME_W, 30, 0xd861);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Manual", REDBUTTON_X + (FRAME_W / 2), REDBUTTON_Y - 15);

  // Hide the Test Button when in Autonomous Mode
 
  //drawManualModeLabel();
  //drawPlusMinusButtons(); // Draw the plus and minus buttons initially
    tft.fillRect(PLUSBUTTON_X + 10, PLUSBUTTON_Y + BUTTON_HEIGHT + 40 , FRAME_W, 40, 0x0841);


     updateButtons(100);
    updateOC(-450);
     // updateGreenSquare(-20);  // Move up by 20 pixels
    drawSetButton();  // Update Set button appearance



}

// Draw a green button (Autonomous Mode)
void greenBtn()
{
  tft.fillRect(GREENBUTTON_X, GREENBUTTON_Y, GREENBUTTON_W, GREENBUTTON_H, 0x0ca6);
  tft.fillRect(REDBUTTON_X, REDBUTTON_Y, REDBUTTON_W, REDBUTTON_H, TFT_DARKGREY);
  drawFrame();
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("M", REDBUTTON_X + (REDBUTTON_W / 2) + 1, REDBUTTON_Y + (REDBUTTON_H / 2));
  SwitchOn = true;

  // Draw the box with text above the button for autonomous mode
  tft.fillRect(REDBUTTON_X, REDBUTTON_Y - 30, FRAME_W, 30, 0x0ca6);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Autonomous", REDBUTTON_X + (FRAME_W / 2), REDBUTTON_Y - 15);
//drawsetbutton
  // Move buttons out of bounds to hide them
  updateButtons(0);
  updateOC(0);
  //drawValueA();
  //updateGreenSquare(20);  // Move down by 20 pixels

 drawPlusMinusButtons();
 drawValue();
     drawSetButton();  // Update Set button appearance


}
void greenBtnClone()
{
  tft.fillRect(GREENBUTTON_X, GREENBUTTON_Y, GREENBUTTON_W, GREENBUTTON_H, 0x4208);
  tft.fillRect(REDBUTTON_X, REDBUTTON_Y, REDBUTTON_W, REDBUTTON_H, 0x7baf);
  drawFrame();
  tft.setTextColor(0xdefb);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("M", REDBUTTON_X + (REDBUTTON_W / 2) + 1, REDBUTTON_Y + (REDBUTTON_H / 2));
  //SwitchOn = true;


  // Draw the box with text above the button for autonomous mode
  tft.fillRect(REDBUTTON_X, REDBUTTON_Y - 30, FRAME_W, 30, 0x6b4d);
  tft.setTextColor(0xdefb);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Disabled!", REDBUTTON_X + (FRAME_W / 2), REDBUTTON_Y - 15);
//drawsetbutton
  // Move buttons out of bounds to hide them
  //updateButtons(0);
 // updateOC(0);
  //drawValueA();
  //updateGreenSquare(20);  // Move down by 20 pixels

 drawPlusMinusButtons();
 
 drawValue();
     drawSetButton();  // Update Set button appearance


}



//middle label
void drawManualModeLabel() {

    tft.fillRect(185, 60, 140, 160, 0x31a6);  // Fill background with the same color as the temp box
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    //tft.drawString("Manual Mode", 160, 190);  // Label text
  
}



void drawValue() {
    sprite.deleteSprite();  // Clear any previous sprite before creating a new one
    sprite.setColorDepth(8);

    sprite.createSprite(80, 40);  // Create a sprite for the value display
    sprite.fillSprite(0x0841);  // Fill with background color

    sprite.loadFont(inter);  // Load Inter font
    sprite.setTextColor(TFT_WHITE);  // Set text color
    sprite.setTextDatum(MC_DATUM);   // Center text inside sprite

    // Draw the current value inside the sprite
    sprite.drawString(String(currentValue), 40, 20);  

    sprite.pushSprite(PLUSBUTTON_X - 77 + (FRAME_W / 2), PLUSBUTTON_Y - 5 + BUTTON_HEIGHT + 50);  // Push to screen

    sprite.unloadFont(); // Unload font to free memory
}

void drawValueRed() {
    sprite.deleteSprite();  // Clear any previous sprite before creating a new one
    sprite.setColorDepth(8);

    sprite.createSprite(80, 40);  // Create a sprite for the value display
    sprite.fillSprite(TFT_RED);  // Fill with background color

    sprite.loadFont(inter);  // Load Inter font
    sprite.setTextColor(TFT_WHITE);  // Set text color
    sprite.setTextDatum(MC_DATUM);   // Center text inside sprite

    // Draw the current value inside the sprite
    sprite.drawString(String(currentValue), 40, 20);  

    sprite.pushSprite(PLUSBUTTON_X - 77 + (FRAME_W / 2), PLUSBUTTON_Y - 5 + BUTTON_HEIGHT + 50);  // Push to screen

    sprite.unloadFont(); // Unload font to free memory
}

/*
void drawValueA() {
   tft.fillRect(FRAME_X + 300, TESTBUTTON_Y + 300 + TESTBUTTON_H + 50, FRAME_W, 40, TFT_BLACK);  // Clear previous value
   tft.setTextColor(TFT_WHITE);
   tft.setTextSize(2);
   tft.setTextDatum(MC_DATUM);
   tft.drawString(String(currentValue), FRAME_X + 300 +  (FRAME_W / 2), TESTBUTTON_Y + TESTBUTTON_H + 700);

   // Draw the Set button
  tft.fillRect(SETBUTTON_X + 300, SETBUTTON_Y + 300, SETBUTTON_W, SETBUTTON_H, TFT_BLUE);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  //tft.drawString("Set", SETBUTTON_X  + 300,  (SETBUTTON_W / 2) + 300 , SETBUTTON_Y + (SETBUTTON_H / 2));  // Label on button

  //draw a black rectangle over the temporary value , will change later 
  tft.fillRect(252 + (FRAME_W / 2) , TESTBUTTON_Y - 20 + TESTBUTTON_H + 70, 200, 33, TFT_BLACK); //replaced frame_x with 290
}
*/

/*
void drawPlusMinusButtons() {
   int radius = 28;  // Radius of the buttons (adjust as needed)

   // Draw minus button as a circle
   tft.fillCircle(MINUSBUTTON_X + radius, MINUSBUTTON_Y + radius, radius, 0x18e3);
   tft.setTextColor(TFT_WHITE);
   tft.setTextSize(2);
   tft.setTextDatum(MC_DATUM);
   tft.drawString("-", MINUSBUTTON_X + radius, MINUSBUTTON_Y + radius);

   // Draw plus button as a circle
   tft.fillCircle(PLUSBUTTON_X + radius, PLUSBUTTON_Y + radius, radius, 0x18e3);
   tft.setTextColor(TFT_WHITE);
   tft.setTextSize(2);
   tft.setTextDatum(MC_DATUM);
   tft.drawString("+", PLUSBUTTON_X + radius, PLUSBUTTON_Y + radius);
}
*/

int getVentAngle(float temperatureF) {
  if (temperatureF <= 55) return 0;    // Vent is fully closed at 55 degrees
  if (temperatureF >= 85) return 180;  // Vent is fully open at 85 degrees 

  // Linear interpolation between 55 and 85 degrees
  return (temperatureF - 55) * (180.0 / (85 - 55));
}

void displayTemperature(float temperatureF) {
    sprite.deleteSprite();  // Clear any previous sprite before creating a new one
    sprite.setColorDepth(8);

    sprite.createSprite(160, 100);  // Maintain the same sprite size
    sprite.fillSprite(0x0841);  // Fill the background with red

    // Load the large font
    sprite.loadFont(med);  

    // Set text properties
    sprite.setTextColor(TFT_WHITE);  // White text for contrast
    sprite.setTextDatum(MC_DATUM);   // Center text in sprite

    // Draw the temperature value inside the sprite
    sprite.drawString(String((int)temperatureF) + "°F", 75, 75);  // Centered in sprite

    // Unload the font to free memory
    sprite.unloadFont();

    // Push the sprite to the screen at the same position
    sprite.pushSprite(190, 50);
}

void displayTemperatureRed(float temperatureF) {
    sprite.deleteSprite();  // Clear any previous sprite before creating a new one
    sprite.setColorDepth(8);

    sprite.createSprite(160, 100);  // Maintain the same sprite size
    sprite.fillSprite(TFT_RED);  // Fill the background with red

    // Load the large font
    sprite.loadFont(med);  

    // Set text properties
    sprite.setTextColor(TFT_WHITE);  // White text for contrast
    sprite.setTextDatum(MC_DATUM);   // Center text in sprite

    // Draw the temperature value inside the sprite
    sprite.drawString(String((int)temperatureF) + "°F", 75, 75);  // Centered in sprite

    // Unload the font to free memory
    sprite.unloadFont();

    // Push the sprite to the screen at the same position
    sprite.pushSprite(190, 50);
}


void displayCO2(uint16_t co2) {
    sprite.deleteSprite();  // Clear any previous sprite before creating a new one
    sprite.setColorDepth(8);
    sprite.createSprite(160, 40);  // Adjust sprite size to fit CO2 text properly
    sprite.fillSprite(0x0841);  //0x0841, Set background (change if necessary)

    // Load the large font
    sprite.loadFont(inter);  

    // Set text properties
    sprite.setTextColor(TFT_WHITE);
    sprite.setTextDatum(MC_DATUM);

    // Draw the CO2 value inside the sprite
    sprite.drawString(String(co2) + " ppm", 76, 25);  // Centered text

    // Unload the font to free memory
    sprite.unloadFont();

    // Push the sprite to the screen at the desired position
    sprite.pushSprite(180, 170);  // Adjust position as needed
}

void displayCO2red(uint16_t co2) {
    sprite.deleteSprite();  // Clear any previous sprite before creating a new one
    sprite.setColorDepth(8);
    sprite.createSprite(160, 40);  // Adjust sprite size to fit CO2 text properly
    sprite.fillSprite(TFT_RED);  //0x0841, Set background (change if necessary)

    // Load the large font
    sprite.loadFont(inter);  

    // Set text properties
    sprite.setTextColor(TFT_WHITE);
    sprite.setTextDatum(MC_DATUM);

    // Draw the CO2 value inside the sprite
    sprite.drawString(String(co2) + " ppm", 76, 25);  // Centered text

    // Unload the font to free memory
    sprite.unloadFont();

    // Push the sprite to the screen at the desired position
    sprite.pushSprite(180, 170);  // Adjust position as needed
}

/*
void temp()
{
  if (millis() - DHT.lastRead() >= 1000)
  {
    //  READ DATA
    uint32_t start = micros();
    int status = DHT.read();
    uint32_t stop = micros();

    if ((count % 10) == 0)
    {
      count = 0;
      Serial.println();
      Serial.println("Type\tHumidity (%)\tTemp (°C)\tTime (µs)\tStatus");
    }
    count++;

    Serial.print("DHT20 \t");
    //  DISPLAY DATA, sensor has only one decimal.
    Serial.print(DHT.getHumidity(), 1);
    Serial.print("\t\t");
    Serial.print(DHT.getTemperature(), 1);
    Serial.print("\t\t");
    Serial.print(stop - start);
    Serial.print("\t\t");
    switch (status)
    {
      case DHT20_OK:
        Serial.print("OK");
        break;
      case DHT20_ERROR_CHECKSUM:
        Serial.print("Checksum error");
        break;
      case DHT20_ERROR_CONNECT:
        Serial.print("Connect error");
        break;
      case DHT20_MISSING_BYTES:
        Serial.print("Missing bytes");
        break;
      case DHT20_ERROR_BYTES_ALL_ZERO:
        Serial.print("All bytes read zero");
        break;
      case DHT20_ERROR_READ_TIMEOUT:
        Serial.print("Read time out");
        break;
      case DHT20_ERROR_LASTREAD:
        Serial.print("Error read too fast");
        break;
      default:
        Serial.print("Unknown error");
        break;
    }
    Serial.print("\n");
  }
}
*/

//breakpoint(2)
// Variable to store the last received voltage value
float lastReceivedVoltage = -1.0; // Initial value that's not a valid voltage

void checkBatteryAndTriggerBuzzer() {
    // Skip if no battery data received yet
    if (BatteryLife.equals("")) {
        return;
    }

    // Convert battery string to float
    float voltage = BatteryLife.toFloat();

    // Print the new voltage reading
    Serial.print("New voltage reading: ");
    Serial.print(voltage);
    Serial.println("V");

    // Store reading in circular buffer
    voltageReadings[currentReadingIndex] = voltage;
    
    // Increment the current reading index and wrap around if necessary
    currentReadingIndex = (currentReadingIndex + 1) % READING_COUNT;

    // Print current buffer contents
    Serial.print("Current Battery Readings: [");
    for (int i = 0; i < READING_COUNT; i++) {
        if (i > 0) Serial.print(", ");
        Serial.print(voltageReadings[i]);
    }
    Serial.println("]");

    // Calculate and display average if enough readings are available
    if (currentReadingIndex == 0) {  // This means the buffer is full
        float sum = 0;
        int validCount = 0;

        // Sum all valid readings
        for (int i = 0; i < READING_COUNT; i++) {
            sum += voltageReadings[i];
            validCount++;
        }

        // Calculate and display average if we have valid readings
        if (validCount > 0) {
            float averageVoltage = sum / validCount;
            Serial.print("Average (");
            Serial.print(validCount);
            Serial.print(" readings): ");
            Serial.print(averageVoltage);
            Serial.println("V");

            // Battery status verification logic
            if (averageVoltage < LOW_BATTERY_THRESHOLD) {
                if (!batteryLowPending && !batteryLow) {
                    // First detection of low voltage
                    batteryLowPending = true;
                    lowBatteryDetectionTime = millis();  // Record detection time
                    Serial.println("Potential low battery, timer will now verify");
                } 
                else if (batteryLowPending && (millis() - lowBatteryDetectionTime >= VERIFICATION_DELAY_MS)) {
                    // Low battery confirmed after verification period
                    batteryLowPending = false;
                    batteryLow = true;
                    Serial.println("CONFIRMED: LOW BATTERY!");
                }
            } 
            
            else {
                // Battery voltage is normal
                if (batteryLowPending) {
                    // Recovered during verification period
                    batteryLowPending = false;
                    Serial.println("False alarm, battery recovered during verification");
                }
                else if (batteryLow) {
                    // Recovered after being confirmed low
                    batteryLow = false;
                    Serial.println("Battery back to normal");
                }
            }
        } 
    } 
    
    else {
        Serial.println("Not enough readings for average yet");
    }
}










//breakpoint 

unsigned long lastSensorReadTime = 0;  // Store last sensor read time
const unsigned long sensorReadInterval = 2000;  // 2 seconds interval for both sensors

unsigned long lastBuzzerUpdate = 0;
int currentFrequency = 4000;
bool increasingFrequency = true;


void displaySensorData() {
    static unsigned long lastSensorReadTime = 0;  // To keep track of when to read the sensors again
    unsigned long currentMillis = millis();  // Current time

    // Only read the sensors every 2 seconds (adjust as necessary)
    if (currentMillis - lastSensorReadTime >= sensorReadInterval) {
        lastSensorReadTime = currentMillis;  // Update the last read time

        // Declare and initialize temperatureF here to make it accessible globally in the function
        //float temperatureF = 0.0;
        //uint16_t co2 = 0;
        float temperature = 0.0;
        float humidity = 0.0;

        // Read the DHT20 sensor (temperature and humidity)
        int status = DHT.read();
        if (status == DHT20_OK) {
            float temperatureC = DHT.getTemperature();  // Read temperature in Celsius
            temperatureF = (temperatureC * 9.0 / 5.0) + 32;  // Convert to Fahrenheit
            humidity = DHT.getHumidity();  // Read humidity

            // Display temperature on the screen
            //displayTemperature(temperatureF);  // Update the temperature display on the screen
        } else {
            Serial.println("DHT20 read failed.");
        }

        // Read the SCD4x CO2 sensor
        uint16_t error;
        bool isDataReady = false;
        
        error = scd4x.getDataReadyFlag(isDataReady);
        if (error) {
            Serial.print("Error trying to execute getDataReadyFlag(): ");
            Serial.println(error);
            return;
        }

        if (isDataReady) {
            error = scd4x.readMeasurement(co2, temperature, humidity);

            if (error) {
                Serial.print("Error reading CO2 sensor: ");
                Serial.println(error);
            } else {
                // Collect the last 5 CO2 readings and calculate the average

                //displayCO2(co2);
                co2Readings[co2Index] = co2;
                co2Index = (co2Index + 1) % 2;    //changed it to 2 from 5

                //changed it to from 5 to 2
                int totalCO2 = 0;
                for (int i = 0; i < 2; i++) {
                    totalCO2 += co2Readings[i];
                }

                // Calculate average CO2 reading
                int avgCO2 = totalCO2 / 2;

                // If the average CO2 level exceeds the threshold, trigger the alarm
if (avgCO2 > 1800) {
    if (!actionTriggered) {
        actionTriggered = true;

        //change this later 
        //isOpen = true;
        openVent();
        ventStatus = "OPEN";
        renderRedScreen();
        
        //drawStatusBox(10, 275, "High CO2 LEVEL!", TFT_RED); //moving the position of this fown fopt now
        //breakpoint b4 this everything works
        
        //Re-render the screen again but make the background red.
        //tft.fillRect(40, 40, 100, 100,TFT_GREEN);

        
        //for top bar

        

        
        if(batteryLow){
          batteryStatusBox(8, 275, "Low Battery!", TFT_ORANGE);
        }

        else if (batteryDead){
          batteryStatusBox(8, 275, "    Battery Dead!", TFT_ORANGE);
        }
/*
        else {
          drawStatusBox(10, 275, "High CO2 LEVEL!", 0x18c3);
        }
*/
        if(!isOpen){
          closeVent();
          ventStatus = "OPEN";
        }

    }
} else {
    if (actionTriggered) {
      connection_status = 100;
      renderScreen();
        actionTriggered = false;
        //renderScreen();
        //drawVentStatus();
        //displayCO2(co2);
    }
}

                // Print all data on the same line
                Serial.print("CO2: ");
                Serial.print(co2);
                Serial.print("\tTemperature: ");
                Serial.print(temperatureF);
                Serial.print(" °F\tAvg CO2: ");
                Serial.print(avgCO2);
                Serial.print("\tVent: ");
                Serial.println(ventStatus);  // Print the vent status
            }
        }

        // Vent control based on temperature and CO2 thresholds
        if (SwitchOn) {  // Autonomous mode - apply threshold logic
            float thresholdHigh = currentValue + 2;
            float thresholdLow = currentValue - 1;

            // Use the temperatureF (Fahrenheit) value for comparison
            if (temperatureF > thresholdHigh && ventStatus != "OPEN") {
                openVent();
                ventStatus = "OPEN";
                drawVentStatus();
                Serial.println("Above Threshold, FBI OPEN UP!");
            } else if (temperatureF < thresholdLow && ventStatus != "CLOSE" && !actionTriggered) {
                closeVent();
                ventStatus = "CLOSE";
                drawVentStatus();
                Serial.println("Below Threshold, Closing");
            }
        }
    }
}







// BuzzerTask: runs in parallel on the ESP32, handling all blocking buzzer calls.
void BuzzerTask(void* pvParameters) {
  for (;;) {
    // 1) If we have batteryLow (and not snoozed), play low battery charm
    if (batteryLow && !snoozeActive) {
      Serial.println("Low Battery Charm Activated (BuzzerTask)...");
      lowBatteryCharm();  // blocking melody
      // Once done, if batteryLow is still set, we’ll repeat next loop iteration.
      // If user set snoozeActive or battery recovers, we skip next time.

      // A small yield
      vTaskDelay(1 / portTICK_PERIOD_MS);
      snoozeActive = true;
    }

    else if (!batteryLow && snoozeActive){
      snoozeActive = false;
    }
    // 2) If high CO2 triggered
    else if (actionTriggered) {
      Serial.println("High CO2 alarm (BuzzerTask)...");
      // The old for-loop with frequency sweep
      for (int i = 4000; i <= 5500; i += 8) {
        ledcAnalogWrite(BUZZER, 200);
        ledcChangeFrequency(BUZZER, i, 12);

        // If actionTriggered gets turned off by main loop, break
        if (!actionTriggered) break;
        // yield
        vTaskDelay(5000);
      }
      // After done, turn off buzzer
      ledcAnalogWrite(BUZZER, 0);
    } else {
      // If neither batteryLow nor actionTriggered, do nothing
      // but don’t hog CPU
      ledcAnalogWrite(BUZZER, 0);
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
  }
}


void print_pinout() {
  Serial.print("TFT_MISO = ");
  Serial.println (TFT_MISO);

  Serial.print("TFT_MOSI = ");
  Serial.println (TFT_MOSI);

  Serial.print("TFT_SCLK = ");
  Serial.println (TFT_SCLK);

  Serial.print("TFT_CS = ");
  Serial.println (TFT_CS);    // Chip select control pin

  Serial.print("TFT_DC = "); 
  Serial.println (TFT_DC);    // (Changed from 2 to 34 Data Command control pin)

  Serial.print("TFT_RST = ");
  Serial.println (TFT_RST);   // Set TFT_RST to -1 if display RESET is connected to ESP32 board RST
}

//start here
void scanI2C() {
    Serial.println("Scanning I2C...");
    byte count = 0;
    for (byte i = 8; i < 120; i++) {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0) {
            Serial.print("Found I2C device at 0x");
            Serial.println(i, HEX);
            count++;
        }
    }
    if (count == 0) Serial.println("No I2C devices found");
}

void lowBatteryNotification() {

    sprite.deleteSprite();  // Clear any previous sprite before creating a new one
    sprite.setColorDepth(8);
    sprite.createSprite(200, 120);  // Adjust sprite size to fit CO2 text properly
    sprite.fillSprite(TFT_WHITE);  //0x0841, Set background (change if necessary)

    // Load the large font
    sprite.loadFont(inter);  

    // Set text properties
    sprite.setTextColor(TFT_BLACK);
    sprite.setTextDatum(MC_DATUM);
    sprite.drawString("Low Battery", 76, 25);  // Centered text

    // Unload the font to free memory
    sprite.unloadFont();

    // Push the sprite to the screen at the desired position
    sprite.pushSprite(170, 90);  // Adjust position as needed
  

}


void drawBreathe(){
  tft.fillRect(0, 0, 177, 40, TFT_BLACK);  //xywh
  tft.setTextDatum(TL_DATUM);        
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.drawString("B.R.E.A.T.H.E", 0, 15);
}


void drawSet(){
  tft.fillRect(390, 0, 177, 40, TFT_BLACK);  //xywh
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(" SET", 396, 15);

}

void renderScreen() {
    tft.fillScreen(0x0841);  // Clear the screen

    sprite.deleteSprite();   // Delete any previous sprite
    sprite.createSprite(320, 40);   // Size to match the "BREATHE Dashboard" section
    sprite.loadFont(inter);         // Load custom font (inter)
    sprite.setTextDatum(4);         // Set text alignment to center

    // Draw BREATHE Dashboard Title
    drawBreathe();

    //Draw Set Dashboard Title
    drawSet();

    // Draw the main content area (normal mode)
    tft.fillRect(0, 40, 155, 300, 0x1082);  // Left side
    tft.fillRect(0, 275, 700, 200, 0x18c3); // Bottom area

    // Draw buttons and values
    greenBtn();
    drawPlusMinusButtons();
    drawValue();
    drawVentStatus();
    displayCO2(co2);
    displayTemperature(temperatureF);

    // **Re-render the battery status box here after the normal screen is restored**
    if (batteryLow) {
        batteryStatusBox(8, 275, "Low Battery!", TFT_ORANGE);  // Low Battery box
    } else if (batteryDead) {
        batteryStatusBox(8, 275, "Battery Dead!", TFT_ORANGE);  // Battery Dead box
    } else {
        emptyBatteryMsg();  // Clear battery box
        batteryStatusBox(270, 275, " ",0x18c3);
    }
}

void renderRedScreen(){


        tft.fillRect(0,40, 155,300, 0x1082);
        tft.fillRect(0,275,700,200, 0x18c3);

        sprite.createSprite(320, 40);   // Size to match the "BREATHE Dashboard" section
        sprite.loadFont(inter);         // Load custom font (inter)
        sprite.setTextDatum(4);         // Set text alignment to center

        // Draw BREATHE Dashboard Title

    // Draw BREATHE Dashboard Title
    //drawBreathe();

    //Draw Set Dashboard Title
    //drawSet();;

        tft.fillRect(156, 40, 900, 234,TFT_RED);

        greenBtnClone();
        drawPlusMinusButtons();
        drawValueRed();
        drawSetButton();  // Update Set button appearance

        drawVentStatus();
        
        displayCO2red(co2);
        displayTemperatureRed(temperatureF);

        // **Re-render the battery status box here after the normal screen is restored**
        if (batteryLow) {
            batteryStatusBox(8, 275, "Low Battery!", TFT_ORANGE);  // Low Battery box
        } else if (batteryDead) {
            batteryStatusBox(8, 275, "Battery Dead!", TFT_ORANGE);  // Battery Dead box
        } else {
            emptyBatteryMsg();  // Clear battery box  // Clear battery box
            batteryStatusBox(270, 275, " ",0x18c3);
        }
}

void setup(void){ 

 // tempSprite.createSprite(160, 100);  // Allocate once and reuse
 // co2Sprite.createSprite(160, 40);

  tft.init();                 //initiliaze tft screen
  tft.setRotation(1);         //set oritentation of screen contetns
  tft.fillScreen(0x0841);  //set screen background to black
  
  print_pinout();
  analogWrite(TFT_BL, 255);   //send a pwm signal to backlight pin
  //digitalWrite(TFT_BL, HIGH);
  Serial.begin(115200);       //set baud rate 
  Serial.println("Starting Vent Control System...");

  BLEDevice::init("ESP32_GUI_Client");
  initializeBLEScan();
  renderScreen();

  //tft.init();                 //initiliaze tft screen
  //tft.setRotation(1);         //set oritentation of screen contetns
  //tft.fillScreen(0x0841);  //set screen background to black
  

  //breakpoint 2
  pinMode(BUZZER, OUTPUT);
  // Setup timer with given frequency, resolution and attach it to a led pin with auto-selected channel

// Set up PWM for the backlight
  ledcAttach(TFT_BL, 10000, LEDC_TIMER_12_BIT);  // 10kHz frequency for backlight
  ledcWrite(TFT_BL, 4095); 

  // Set up PWM for the buzzer
  ledcAttach(BUZZER, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);  // Set buzzer frequency
  ledcWrite(BUZZER, 0);  //initially have the buzzer off. 
  
  // Initialize SCD4x sensor
  Wire.begin();
  scd4x.begin(Wire);
  scanI2C();
  DHT.begin();

      uint16_t error = scd4x.startPeriodicMeasurement();
    if (error) {
        Serial.println("Failed to start SCD41 measurements.");
    } else {
        Serial.println("SCD41 measurement started.");
    }
/*
  sprite.createSprite(320, 40);   // Size to match the "BREATHE Dashboard" section
  sprite.loadFont(inter);         // Load custom font (inter)
  sprite.setTextDatum(4);         // Set text alignment to center

  // Draw BREATHE Dashboard Title
  sprite.fillSprite(TFT_BLACK);   
  sprite.setTextColor(TFT_WHITE);
  sprite.setTextSize(1);  
  sprite.drawString("B.R.E.A.T.H.E", 78, 24);  
  sprite.pushSprite(0, 0);  

  // Draw G40 On Dashboard TItle
  sprite.fillSprite(TFT_BLACK);
  sprite.setTextColor(TFT_WHITE);
  sprite.drawString(" SET", 105, 24);
  sprite.pushSprite(320, 0); 

  tft.fillRect(0,40, 155,300, 0x1082);
  tft.fillRect(0,275,700,200, 0x18c3); //0x18c3
  //drawOpenCloseButtons();
  //drawStatusBox(" ", 0x18c3);
  //tft.fillRect(10,275, 300, 300, TFT_GREEN);


    




  //draw initial status boxes before data is sent to them
  //drawStatusBox(100, 60, "Temp: -- °C", 0x31a6);
  //drawStatusBox(10, 130, "CO2: -- ppm", 0x31a6);  // CO2 on the left side

  //by default, initiliaze with autonomous mode
  greenBtn();  
  //drawGreenSquare();
  // pinMode(BUZZER, OUTPUT);
  drawValue();
  drawVentStatus();
  //lowBatteryNotification();
  */

/*
        sprite.fillSprite(TFT_BLACK); // Clear previous value
        sprite.setTextColor(TFT_WHITE);
        sprite.drawString("test", 10, 30); // Display the current number
        sprite.pushSprite(80, 80);  // Push the updated sprite to the screen
        */
  //sprite.drawString("Test", 120, 50);
  //sprite.pushSprite(0, 0);

  //tft.fillRect(156, 40, 900, 234,TFT_GREEN);
  esp_err_t errRc=esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT,ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN ,ESP_PWR_LVL_P9);

  // --- FreeRTOS: Create BuzzerTask pinned to core 1 (or 0)
  xTaskCreatePinnedToCore(
    BuzzerTask,      // function
    "BuzzerTask",    // name
    4096,            // stack size
    NULL,            // param
    1,               // priority
    &BuzzerTaskHandle,
    1                // run on core 1 (or 0)
  );

}


void loop() {
  //lowBatteryCharm();
  //print_pinout();
  uint16_t x, y;
  static unsigned long lastDebounceTime = 0;  // For debouncing
  unsigned long debounceDelay = 30;  // 50ms debounce time
  displaySensorData();
  //temp();
  lame();
  delay(100); // Reduce the delay for better responsiveness
  handleBluetooth();
  Serial.print("Free Heap: ");
  Serial.println(ESP.getFreeHeap());

  
    // Display "Connected" or "Not connected" below the OPEN button
    /*
    if (isConnected) {
        tft.setCursor(10, 260);  // Position below OPEN button
        tft.print("Connected");
    } else {
        tft.setCursor(10, 260);  // Position below OPEN button
        tft.print("Not connected");
    }
    */

  if (!actionTriggered){
    displayCO2(co2);
    displayTemperature(temperatureF);
  }
  else if (actionTriggered) {
    displayCO2red(co2);
    displayTemperatureRed(temperatureF);
        for (int i = 4000; i <= 5500; i += 8) {
            ledcAnalogWrite(BUZZER, 200);
            ledcChangeFrequency(BUZZER, i, 12);
           // Serial.print("Buzzer frequency: ");
           // Serial.println(i);
            delayMicroseconds(5000);
            
            // Check for exit condition
            if (!actionTriggered) break;
            
        }
        
        // Reset frequency when done
        if (!actionTriggered) {
            ledcAnalogWrite(BUZZER, 0);
            
        } else {
            // Immediately start next sweep
            ledcAnalogWrite(BUZZER, 0);
            //displayCO2(co2);
        }
    }

    
    
    
    
/*
    if(batteryLow) {
      lowBatteryCharm();
      Serial.println("Low Battery Charm Activated");
    
        if (!batteryLow) {
            ledcAnalogWrite(BUZZER, 0);
        } else {
            // Immediately start next sweep
            ledcAnalogWrite(BUZZER, 0);
        }
    }
    
    */




  //checkBatteryAndTriggerBuzzer();
 // Serial.println(BatteryLife);
 
 /*
  Serial.print("Average Voltage: ");
  Serial.println(averageVoltage);  // Print the most recent average voltage
 /*
  if (batteryLow) {
    Serial.println("Battery is LOW!");  // Print low battery status
  } else {
    Serial.println("Battery is NORMAL");  // Print normal battery status
  }
  */
/*
  
if (batteryLow && !snoozeActive) {
    int buzzerFrequency = ledcReadFreq(BUZZER); // Read the frequency
Serial.print("Buzzer Frequency: ");
Serial.println(buzzerFrequency); // Print the buzzer frequency
    // Handle low battery song play
    // Check if touch is detected first
    uint16_t x, y;
    if (tft.getTouch(&x, &y)) {
        // Check if snooze button is pressed only when battery is low
        if ((x > SNOOZEBUTTON_X) && (x < (SNOOZEBUTTON_X + SNOOZEBUTTON_W))) {
            if ((y > SNOOZEBUTTON_Y) && (y <= (SNOOZEBUTTON_Y + SNOOZEBUTTON_H))) {
                Serial.println("Snooze button pressed");
                ledcWrite(BUZZER, 0);  // Initially have the buzzer off.
                snoozeActive = true;  // Turn off the buzzer when snooze button is pressed
                Serial.println(snoozeActive);
                tft.fillRect(SNOOZEBUTTON_X, SNOOZEBUTTON_Y, SNOOZEBUTTON_W, SNOOZEBUTTON_Y, TFT_ORANGE);
                
            }
        }
    }
}
  // TODO: implement logic if a snooze is off
  else if (!batteryLow) {
    song_pt = 1;  // Reset song
    snoozeActive = false;
    Serial.println(snoozeActive);
  }
*/

  // Handle snooze and buzzer logic
  //handleSnooze();  // Continuously check and handle snooze functionality
 

  // See if there's any touch data for us
  if (tft.getTouch(&x, &y)) {
    // Check for debounce
    if ((millis() - lastDebounceTime) > debounceDelay) {
      lastDebounceTime = millis();  // Record the time of the last valid press

      // Draw a block spot to show where touch was calculated to be
      #ifdef BLACK_SPOT
        tft.fillCircle(x, y, 2, TFT_BLACK);
      #endif

        

  //breakpoint 5


      
      if (SwitchOn) {
        if ((x > REDBUTTON_X) && (x < (REDBUTTON_X + REDBUTTON_W))) {
          if ((y > REDBUTTON_Y) && (y <= (REDBUTTON_Y + REDBUTTON_H))) {
            Serial.println("Red btn hit");
            redBtn();
          }
        }
      } 
      
      else {  // Record is off (SwitchOn == false)
        if ((x > GREENBUTTON_X) && (x < (GREENBUTTON_X + GREENBUTTON_W))) {
          if ((y > GREENBUTTON_Y) && (y <= (GREENBUTTON_Y + GREENBUTTON_H))) {
            Serial.println("Green btn hit");
            greenBtn();
          }
        }
      }

      // Test button action (only in Manual mode)
      if (SwitchOn && (x > TESTBUTTON_X) && (x < (TESTBUTTON_X + TESTBUTTON_W))) {
        if ((y > TESTBUTTON_Y) && (y <= (TESTBUTTON_Y + TESTBUTTON_H))) {
          Serial.println("Test Button hit");
        }
      }
      Serial.println(SwitchOn);

      if ((x > MINUSBUTTON_X) && (x < (MINUSBUTTON_X + BUTTON_WIDTH))) {
        if ((y > MINUSBUTTON_Y) && (y <= (MINUSBUTTON_Y + BUTTON_HEIGHT))) {
          if (currentValue > 55) {
            currentValue--;
            drawValue();
          }
        }
      }

      if ((x > PLUSBUTTON_X) && (x < (PLUSBUTTON_X + BUTTON_WIDTH))) {
        if ((y > PLUSBUTTON_Y) && (y <= (PLUSBUTTON_Y + BUTTON_HEIGHT))) {
          if (currentValue < 85) {
            currentValue++;
            drawValue();
          }
        }
      }

      if ((x > SETBUTTON_X) && (x < (SETBUTTON_X + SETBUTTON_W))) {
              if ((y > SETBUTTON_Y) && (y <= (SETBUTTON_Y + SETBUTTON_H))) {
                float currentTemperatureF = (float)currentValue;  // Use the currentValue as the temperature in Fahrenheit
                
                // Get the vent angle corresponding to the temperature
                int ventAngle = getVentAngle(currentTemperatureF);

                // Print the temperature and vent angle to the Serial Monitor
                /*
                Serial.print("Temperature: ");
                Serial.print(currentTemperatureF);
                Serial.print(" °F, Vent Angle: ");
                Serial.println(ventAngle);
                */
          }
        }

            if ((x > SETBUTTON_X) && (x < (SETBUTTON_X + SETBUTTON_W))) {
                if ((y > SETBUTTON_Y) && (y <= (SETBUTTON_Y + SETBUTTON_H))) {
                    if (SwitchOn) {  // Only allow in Autonomous Mode
                        Serial.print("Sending temperature setting: ");
                        Serial.println(currentValue);
                        sendTemperatureValue();  // Send value over BLE
                    }
                }
            }

      if ((x > OPENBUTTON_X) && (x < (OPENBUTTON_X + BUTTON_W)) &&
          (y > OPENBUTTON_Y) && (y <= (OPENBUTTON_Y + BUTTON_H))) {
          Serial.println("Vent Status: OPEN");
          ventStatus = "OPEN";  // Update status
          drawVentStatus();  // Refresh text
          openVent();
      }

// Close Button Pressed
      if ((x > CLOSEBUTTON_X) && (x < (CLOSEBUTTON_X + BUTTON_W)) &&
          (y > CLOSEBUTTON_Y) && (y <= (CLOSEBUTTON_Y + BUTTON_H))) {
          Serial.println("Vent Status: CLOSE");
          ventStatus = "CLOSE";  // Update status
          drawVentStatus();  // Refresh text
          closeVent();
      }
//snooze button

    }
  
  // Continuously display sensor data

  }
/*
    if (!isConnected) {
        Serial.println("Scanning for Vent...");
        startBLEScan();
    }
    */
}

//please work!