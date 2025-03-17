/************************************************************
*
*   B.R.E.A.T.H.E Control and Sensor Sub System Code
*   Written By Patrick, assisted by Marcus, Emily, and Kyle
*T
************************************************************/

//original spot 
//most updated code as of 3/14/2025, 7:17

#include <SPI.h>                          //include necessary libraries such as spi, i2c, and tft
#include <TFT_eSPI.h>           
#include <Wire.h>
#include <SensirionI2CScd4x.h>
#include "inter.h"                        //define font library
//#include "interlarge.h"
#include "large.h"
#include "small.h"

#include "DHT20.h"

//new beginning

//1 - include BLE libraries
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define TFT_BL 17                         //define backlight pin   
#define BUZZER 33                          //define buzzer pin                
#define TP_BUSY 2
#define TP_CS 13
#define TP_IRQ 12


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
#define quarter_note_length 300
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

String ventStatus = "OPEN";  // Default status is OPEN

#define VENT_SERVICE_UUID         "12345678-1234-5678-1234-56789abcdef0"  // Replace with actual UUID
#define VENT_CHARACTERISTIC_UUID  "abcdef01-1234-5678-1234-56789abcdef1"  // Replace with actual UUID
#define BATTERY_LIFE_CHARACTERISTIC_UUID      "e540f441-a246-4072-b162-e2b67d4c7f57"
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

String BatteryLife = "";

static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    Serial.print(" Notification received: ");
    Serial.println((char*)pData);
    BatteryLife = (char*)pData;
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

void startBLEScan() {
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(5, false);
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
    sendVentCommand("open");
}

void closeVent() {
    sendVentCommand("close");
}

//breakpoint 1000

void ledcAnalogWrite(uint8_t pin, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 4095 from 2 ^ 12 - 1
  uint32_t duty = (4095 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWrite(pin, duty);
}

//for low battery sound:
//-------------------------------------------------------------------//
void restNote(int time) {
  analogWrite(BUZZER, 0);
  delay(time);
  analogWrite(BUZZER, ONVALUE);
}

void playNote(note_t note, int timeLength, int octave) {
  // play the note for 90% of the time then turn off for a little
  ledcWriteNote(BUZZER, note, octave);
  delay(timeLength * 0.9);
  restNote(timeLength * 0.1);
}

void playNote_legato(note_t note, int timeLength, int octave) {
  ledcWriteNote(BUZZER, note, octave);
  delay(timeLength);
}

/*
void lowBatteryCharm() {
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

  restNote(5000);
}
*/
//-------------------------------------------------------------------//


void handleBluetooth() {
    static bool wasConnected = false; // Tracks previous connection status

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
        BLEDevice::getScan()->start(0);
        wasConnected = false; // Update status so it prints again when reconnected
    }
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
    sprite.createSprite(SETBUTTON_W, SETBUTTON_H);  // Create sprite for button area

    if (SwitchOn) {  // Autonomous Mode
        sprite.fillSprite(0x4208);  // Green background for Autonomous mode

        sprite.loadFont(small);  
        sprite.setTextColor(TFT_WHITE, 0x4208);  // White text for contrast
        sprite.setTextDatum(MC_DATUM);
        sprite.drawString("Set", SETBUTTON_W / 2, SETBUTTON_H / 2);
        sprite.unloadFont();
    } 
    else {  // Manual Mode (Disabled)
        sprite.fillSprite(0x1082);  // Gray background for Manual mode

        sprite.loadFont(small);
        sprite.setTextColor(0x4228, 0x1082);  // Darker text for contrast
        sprite.setTextDatum(MC_DATUM);
        sprite.drawString("Disabled", SETBUTTON_W / 2, SETBUTTON_H / 2);
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



// Function to draw a standard status box
void drawStatusBox(int x, int y, String text, uint16_t color) {
    sprite.deleteSprite();  // Clear any previous sprite before creating a new one

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

  // Move buttons out of bounds to hide them
  updateButtons(0);
  updateOC(0);
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

    sprite.createSprite(175, 100);  // Maintain the same sprite size
    sprite.fillSprite(0x0841);  // Fill the background with red

    // Load the large font
    sprite.loadFont(large);  

    // Set text properties
    sprite.setTextColor(TFT_WHITE);  // White text for contrast
    sprite.setTextDatum(MC_DATUM);   // Center text in sprite

    // Draw the temperature value inside the sprite
    sprite.drawString(String((int)temperatureF) + "°", 75, 75);  // Centered in sprite

    // Unload the font to free memory
    sprite.unloadFont();

    // Push the sprite to the screen at the same position
    sprite.pushSprite(190, 50);
}


void displayCO2(uint16_t co2) {
    sprite.deleteSprite();  // Clear any previous sprite before creating a new one
    sprite.createSprite(185, 40);  // Adjust sprite size to fit CO2 text properly
    sprite.fillSprite(0x0841);  //0x0841, Set background (change if necessary)

    // Load the large font
    sprite.loadFont(inter);  

    // Set text properties
    sprite.setTextColor(TFT_WHITE);
    sprite.setTextDatum(MC_DATUM);

    // Draw the CO2 value inside the sprite
    sprite.drawString(String(co2) + " ppm", 77, 25);  // Centered text

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

void checkBatteryAndTriggerBuzzer() {
    // turn string into float
    if (BatteryLife.equals("")) {
      return;
    }
    float batteryLifeValue = BatteryLife.toFloat();  // Convert the battery life to float
    
    //breakpoint
    //when battery is plugged in change to 1.20
    if (batteryLifeValue < 1.20) {
        // Turn the buzzer on
        //analogWrite(BUZZER, 100);  // Adjust value if using PWM to control buzzer
        //lowBatteryCharm();
        //Serial.println("Battery life is low, buzzer ON!");
        //Serial.println(BatteryLife);
        drawStatusBox(270, 275, "Low Battery!", TFT_RED);
    } else {
        // Turn the buzzer off
        analogWrite(BUZZER, 0);  // Turn off buzzer
        //Serial.println("Battery life is sufficient, buzzer OFF.");
        //Serial.println(BatteryLife);
        drawStatusBox(270, 275, " ", 0x18c3);
    }
}

//breakpoint 

unsigned long lastSensorReadTime = 0;  // Store last sensor read time
const unsigned long sensorReadInterval = 2000;  // 2 seconds interval for both sensors
void displaySensorData() {
    static unsigned long lastSensorReadTime = 0;  // To keep track of when to read the sensors again
    unsigned long currentMillis = millis();  // Current time

    // Only read the sensors every 2 seconds (adjust as necessary)
    if (currentMillis - lastSensorReadTime >= sensorReadInterval) {
        lastSensorReadTime = currentMillis;  // Update the last read time

        // Declare and initialize temperatureF here to make it accessible globally in the function
        float temperatureF = 0.0;
        uint16_t co2 = 0;
        float temperature = 0.0;
        float humidity = 0.0;

        // Read the DHT20 sensor (temperature and humidity)
        int status = DHT.read();
        if (status == DHT20_OK) {
            float temperatureC = DHT.getTemperature();  // Read temperature in Celsius
            temperatureF = (temperatureC * 9.0 / 5.0) + 32;  // Convert to Fahrenheit
            humidity = DHT.getHumidity();  // Read humidity

            // Display temperature on the screen
            displayTemperature(temperatureF);  // Update the temperature display on the screen
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

                displayCO2(co2);
                co2Readings[co2Index] = co2;
                co2Index = (co2Index + 1) % 5;

                int totalCO2 = 0;
                for (int i = 0; i < 5; i++) {
                    totalCO2 += co2Readings[i];
                }

                // Calculate average CO2 reading
                int avgCO2 = totalCO2 / 5;

                // If the average CO2 level exceeds the threshold, trigger the alarm
                if (avgCO2 > 1800) {
                  if (!actionTriggered) {
                      actionTriggered = true;
                      drawStatusBox(10, 275, "High CO2 LEVEL!", TFT_RED);
                      analogWrite(BUZZER, 100);
                      /*
                      for (int i = 4000; i <= 5500; i+=8) {
                          ledcAnalogWrite(BUZZER, 200);
                          ledcChangeFrequency(BUZZER, i, 12); // 1 khz, 12 bit resolution
                          delayMicroseconds(8000);
                      }
                      */
                  } 
                } else {
                    if (actionTriggered) {
                        actionTriggered = false;
                        drawStatusBox(10, 275, "Normal CO2 Levels :)", 0x18c3);  // Display normal CO2 message
                        analogWrite(BUZZER, 0); // Turn off buzzer or LED
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
            } else if (temperatureF < thresholdLow && ventStatus != "CLOSED") {
                closeVent();
                ventStatus = "CLOSED";
                drawVentStatus();
                Serial.println("Below Threshold, Closing");
            }
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


void setup(void){ 
  print_pinout();
  analogWrite(TFT_BL, 255);   //send a pwm signal to backlight pin
  //digitalWrite(TFT_BL, HIGH);
  Serial.begin(115200);       //set baud rate 
  Serial.println("Starting Vent Control System...");

    BLEDevice::init("ESP32_GUI_Client");
    startBLEScan();

  tft.init();                 //initiliaze tft screen
  tft.setRotation(1);         //set oritentation of screen contetns
  tft.fillScreen(0x0841);  //set screen background to black

  //breakpoint 2
    pinMode(BUZZER, OUTPUT);
  // Setup timer with given frequency, resolution and attach it to a led pin with auto-selected channel
  ledcAttach(BUZZER, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcAttach(TFT_BL, BL_FREQ, LEDC_TIMER_12_BIT);
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
  sprite.drawString("SD G40", 105, 24);
  sprite.pushSprite(320, 0); 

  tft.fillRect(0,40, 155,300, 0x1082);
  tft.fillRect(0,275,700,200, 0x18c3); //0x18c3
  //drawOpenCloseButtons();



    




  //draw initial status boxes before data is sent to them
  //drawStatusBox(100, 60, "Temp: -- °C", 0x31a6);
  //drawStatusBox(10, 130, "CO2: -- ppm", 0x31a6);  // CO2 on the left side

  //by default, initiliaze with autonomous mode
  greenBtn();  
  //drawGreenSquare();
  // pinMode(BUZZER, OUTPUT);
  drawValue();
  drawVentStatus();


/*
        sprite.fillSprite(TFT_BLACK); // Clear previous value
        sprite.setTextColor(TFT_WHITE);
        sprite.drawString("test", 10, 30); // Display the current number
        sprite.pushSprite(80, 80);  // Push the updated sprite to the screen
        */
  //sprite.drawString("Test", 120, 50);
  //sprite.pushSprite(0, 0);

  


}

void loop() {
  //lowBatteryCharm();
  uint16_t x, y;
  static unsigned long lastDebounceTime = 0;  // For debouncing
  unsigned long debounceDelay = 30;  // 50ms debounce time
  displaySensorData();
  //temp();
  delay(100); // Reduce the delay for better responsiveness
  handleBluetooth();
  checkBatteryAndTriggerBuzzer();
 // Serial.println(BatteryLife);


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
          Serial.println("Vent Status: CLOSED");
          ventStatus = "CLOSED";  // Update status
          drawVentStatus();  // Refresh text
          closeVent();
      }
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
