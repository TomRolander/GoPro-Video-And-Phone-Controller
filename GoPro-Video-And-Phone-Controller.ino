/**************************************************************************
  GoPro Video and Phone Controller

  Original Code:  2022-03-10

  Tom Rolander, MSEE
  Mentor, Circuit Design & Software
  Miller Library, Fabrication Lab
  Hopkins Marine Station, Stanford University,
  120 Ocean View Blvd, Pacific Grove, CA 93950
  +1 831.915.9526 | rolander@stanford.edu

 **************************************************************************/

/**************************************************************************

  To Do:
  - Adjust time for DST

 **************************************************************************/

#define PROGRAM "GoPro Video and Phone Controller"
#define VERSION "Ver 0.1 2022-03-10"

#define DEBUG_OUTPUT 1

#include "BLEDevice.h"
#include "BLEScan.h"
#include <dummy.h>
#include <Arduino.h>

#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <WiFi.h>
#include "time.h"
#include <SoftwareSerial.h>

const char* ssid     = "Stanford";                    // your network SSID (name)
const char* password = "";                    // your network password

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -(8*3600);
int   daylightOffset_sec = 3600;  //  = 0;


SoftwareSerial espSerialPhone(19,18);

//BLE Connection variables

#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *pServer;
BLEService *pService;
BLECharacteristic *pTxCharacteristic;
BLECharacteristic *pRxCharacteristic;

bool deviceConnected = false;
std::string rxValue = "";

bool bluefruitconnectstartrecording = false;
bool bluefruitconnectstoprecording = false;

// GoPro Service UUID
static BLEUUID serviceUUID_FEA6("0000fea6-0000-1000-8000-00805f9b34fb");

// GoPro Command UUIDs
static BLEUUID    commandUUID("b5f90072-aa8d-11e3-9046-0002a5d5c51b");
static BLEUUID    commandResponseUUID("b5f90073-aa8d-11e3-9046-0002a5d5c51b");

// GoPro Settings UUIDs
static BLEUUID    settingsUUID("b5f90074-aa8d-11e3-9046-0002a5d5c51b");
static BLEUUID    settingsResponseUUID("b5f90075-aa8d-11e3-9046-0002a5d5c51b");

// GoPro Query UUIDs
static BLEUUID    queryUUID("b5f90076-aa8d-11e3-9046-0002a5d5c51b");
static BLEUUID    queryResponseUUID("b5f90077-aa8d-11e3-9046-0002a5d5c51b");

static boolean foundBLEService = false;
static boolean connectedBLE = false;
static boolean waitForCommandNotify = false;
static boolean waitForSettingsNotify = false;
static boolean waitForQueryNotify = false;
static boolean doScan = false;

static BLERemoteCharacteristic* pCommandCharacteristic;
static BLERemoteCharacteristic* pCommandResponseCharacteristic;
static BLERemoteCharacteristic* pSettingsCharacteristic;
static BLERemoteCharacteristic* pSettingsResponseCharacteristic;
static BLERemoteCharacteristic* pQueryCharacteristic;
static BLERemoteCharacteristic* pQueryResponseCharacteristic;

static BLEAdvertisedDevice* myDevice;

static long keepAliveTicker = 0;
static long ticker = 0;
static bool onOffLED = false;

static uint8_t notifyCommandData[128];
static size_t  notifyCommandLength;
static uint8_t notifySettingsData[128];
static size_t  notifySettingsLength;
static uint8_t notifyQueryData[128];
static size_t  notifyQueryLength;

static void notifyCommandCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) 
{
//  Serial.print("Notify callback set");
  Serial.print("Notify callback for command characteristic ");
  Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  Serial.println(" of:");
  Serial.print("  data length ");
  Serial.print(length);
  notifyCommandLength = length;
  Serial.print(" data:");
  for (int i=0; i<length; i++)
  {
    Serial.print(" ");
    Serial.print(pData[i], HEX);
    notifyCommandData[i] = pData[i];
  }
  Serial.println("");

  if ((strcmp(pBLERemoteCharacteristic->getUUID().toString().c_str(), "b5f90073-aa8d-11e3-9046-0002a5d5c51b") == 0)
    && pData[2] == 0)
  {
    Serial.println("Command sent successfully");
    waitForCommandNotify = true;
  }
  else
  {
    Serial.println("Unexpected response");
  }
}

static void notifySettingsCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) 
{
//  Serial.print("Notify callback set");
  Serial.print("Notify callback for settings characteristic ");
  Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  Serial.println(" of:");
  Serial.print("  data length ");
  Serial.print(length);
  notifySettingsLength = length;
  Serial.print(" data:");
  for (int i=0; i<length; i++)
  {
    Serial.print(" ");
    Serial.print(pData[i], HEX);
    notifySettingsData[i] = pData[i];
  }
  Serial.println("");

  if ((strcmp(pBLERemoteCharacteristic->getUUID().toString().c_str(), "b5f90075-aa8d-11e3-9046-0002a5d5c51b") == 0)
    && pData[2] == 0)
  {
    Serial.println("Settings sent successfully");
    waitForSettingsNotify = true;
  }
  else
  {
    Serial.println("Unexpected response");
  }
}

static void notifyQueryCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) 
{
//  Serial.print("Notify callback set");
  Serial.print("Notify callback for query characteristic ");
  Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  Serial.println(" of:");
  Serial.print("  data length ");
  Serial.print(length);
  notifyQueryLength = length;
  Serial.print(" data:");
  for (int i=0; i<length; i++)
  {
    Serial.print(" ");
    Serial.print(pData[i], HEX);
    notifyQueryData[i] = pData[i];
  }
  Serial.println("");

  if ((strcmp(pBLERemoteCharacteristic->getUUID().toString().c_str(), "b5f90077-aa8d-11e3-9046-0002a5d5c51b") == 0)
    && pData[2] == 0)
  {
    Serial.println("Query sent successfully");
    waitForQueryNotify = true;
  }
  else
  {
    Serial.println("Unexpected response");
  }
}

class MyClientCallback : public BLEClientCallbacks 
{
    void onConnect(BLEClient* pclient) 
    {
      Serial.print("MyClientCallback -> onConnect");
    }

    void onDisconnect(BLEClient* pclient) 
    {
      connectedBLE = false;
      Serial.print("MyClientCallback -> onDisconnect");
      failedLED();
    }
};

bool connectToBLEServer() 
{
  Serial.print("Forming connection:");
  Serial.println(myDevice->getAddress().toString().c_str());

  //Set GoPro encryption (this is necessary for proper connection)
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);

  BLEClient*  pClient  = BLEDevice::createClient();

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remove BLE Server.
  pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  Serial.println("Connected to server");

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService_FEA6 = pClient->getService(serviceUUID_FEA6);
  if (pRemoteService_FEA6 == nullptr) {
    Serial.println("Failed to find FEA6 service UUID");
    pClient->disconnect();
    return false;
  }
  Serial.println("FOUND FEA6 service UUID");


  // Obtain a reference to the command characteristic in the service of the remote BLE server.
  pCommandCharacteristic = pRemoteService_FEA6->getCharacteristic(commandUUID);
  if (pCommandCharacteristic == nullptr) {
    Serial.println("Failed to find command characteristic UUID");
    pClient->disconnect();
    return false;
  }
  Serial.println("Found command characteristic");

// Verify that can write command characteristic
  if (pCommandCharacteristic->canWrite()) 
  {
    Serial.println("Confirm that can write command characteristic");
  }
  else
  {
    Serial.println("FAILED to Confirm that can write command characteristic");
    pClient->disconnect();
    return false;
  }

  // Obtain a reference to the command response characteristic in the service of the remote BLE server.
  pCommandResponseCharacteristic = pRemoteService_FEA6->getCharacteristic(commandResponseUUID);
  if (pCommandResponseCharacteristic == nullptr) {
    Serial.println("Failed to find command response characteristic UUID");
    pClient->disconnect();
    return false;
  }
  Serial.println("Found command response characteristic");

// Verify that can notify command response characteristic.
  if (pCommandResponseCharacteristic->canNotify())
  {
    Serial.println("Confirm that can notify command response characteristic");
    pCommandResponseCharacteristic->registerForNotify(notifyCommandCallback);
  }
  else
  {
    Serial.println("FAILED to Confirm that can notify command response characteristic");
    pClient->disconnect();
    return false;
  }


  // Obtain a reference to the settings characteristic in the service of the remote BLE server.
  pSettingsCharacteristic = pRemoteService_FEA6->getCharacteristic(settingsUUID);
  if (pSettingsCharacteristic == nullptr) {
    Serial.println("Failed to find settings characteristic UUID");
    pClient->disconnect();
    return false;
  }
  Serial.println("Found settings characteristic");

// Verify that can write settings characteristic
  if (pSettingsCharacteristic->canWrite()) 
  {
    Serial.println("Confirm that can write settings characteristic");
  }
  else
  {
    Serial.println("FAILED to Confirm that can write settings characteristic");
    pClient->disconnect();
    return false;
  }

  // Obtain a reference to the settings response characteristic in the service of the remote BLE server.
  pSettingsResponseCharacteristic = pRemoteService_FEA6->getCharacteristic(settingsResponseUUID);
  if (pSettingsResponseCharacteristic == nullptr) {
    Serial.println("Failed to find settings response characteristic UUID");
    pClient->disconnect();
    return false;
  }
  Serial.println("Found settings response characteristic");

// Verify that can notify settings response characteristic.
  if (pSettingsResponseCharacteristic->canNotify())
  {
    Serial.println("Confirm that can notify settings response characteristic");
    pSettingsResponseCharacteristic->registerForNotify(notifySettingsCallback);
  }
  else
  {
    Serial.println("FAILED to Confirm that can notify settings response characteristic");
    pClient->disconnect();
    return false;
  }
   
  
  // Obtain a reference to the query characteristic in the service of the remote BLE server.
  pQueryCharacteristic = pRemoteService_FEA6->getCharacteristic(queryUUID);
  if (pQueryCharacteristic == nullptr) {
    Serial.println("Failed to find query characteristic UUID");
    pClient->disconnect();
    return false;
  }
  Serial.println("Found query characteristic");

// Verify that can write query characteristic
  if (pQueryCharacteristic->canWrite()) 
  {
    Serial.println("Confirm that can write query characteristic");
  }
  else
  {
    Serial.println("FAILED to Confirm that can write query characteristic");
    pClient->disconnect();
    return false;
  }

  // Obtain a reference to the query response characteristic in the service of the remote BLE server.
  pQueryResponseCharacteristic = pRemoteService_FEA6->getCharacteristic(queryResponseUUID);
  if (pQueryResponseCharacteristic == nullptr) {
    Serial.println("Failed to find query response characteristic UUID");
    pClient->disconnect();
    return false;
  }
  Serial.println("Found query response characteristic");

// Verify that can notify query response characteristic.
  if (pQueryResponseCharacteristic->canNotify())
  {
    Serial.println("Confirm that can notify query response characteristic");
    pQueryResponseCharacteristic->registerForNotify(notifyQueryCallback);
  }
  else
  {
    Serial.println("FAILED to Confirm that can notify query response characteristic");
    pClient->disconnect();
    return false;
  }
   

  connectedBLE = true;
  return true;
}
/**
   Scan for BLE servers and find the first one that advertises the service we are looking for.
*/
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    /**
        Called for each advertising BLE server.
    */
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.print("BLE Advertised Device found: ");
      Serial.println(advertisedDevice.toString().c_str());

      // We have found a device, let us now see if it contains the service we are looking for.
      if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID_FEA6)) {

Serial.println("FOUND service!");

        BLEDevice::getScan()->stop();
        myDevice = new BLEAdvertisedDevice(advertisedDevice);
        foundBLEService = true;
        doScan = true;

      } // Found our server
    } // onResult
}; // MyAdvertisedDeviceCallbacks

void failedLED()
{
  //Failed to take photo
  for (int i = 0; i <= 5; i++)
  {
    digitalWrite(2, HIGH);
    delay(200);
    digitalWrite(2, LOW);
    delay(200);
  }
}

void ble_send(std::string msg) {
//  Serial.print("sending: ");
//  Serial.println(msg.c_str());
  pTxCharacteristic->setValue((byte*)msg.c_str(), msg.size());
  pTxCharacteristic->notify();
}

void on_ble_receive(std::string msg) 
{
  // Process the command
  char strCommand[32] = "";
  int iCommand = 0;
  strcpy(strCommand, msg.c_str());

#if DEBUG_OUTPUT
Serial.print("strCommand = ");
Serial.print(strCommand);
#endif

  espSerialPhone.print(strCommand);
}

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        Serial.println("Smartphone CONNECTED with BlueFruitConnect");
        deviceConnected = true;
    }
    void onDisconnect(BLEServer *pServer)
    {
        Serial.println("Smartphone DISCONNECTED with BlueFruitConnect");
        deviceConnected = false;
    }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        Serial.print("received: ");
        rxValue = pCharacteristic->getValue();
        Serial.println(rxValue.c_str());
        on_ble_receive(rxValue);
    }
};


void setup() {
  Serial.begin(115200);
  delay(2000);

  espSerialPhone.begin(600);
  
  Serial.println("");
  Serial.println(PROGRAM);
  Serial.println(VERSION);  

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

#if 1
  Serial.println("Starting WiFi client...");
    Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println(" CONNECTED");
  
  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
#endif

  BLEDevice::init("ESP32 GoPro");

  Serial.println("Starting BLE Client...");

  Serial.println("Launch BluefruitConnect on your iPhone or Android"); 
   
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  pService = pServer->createService(SERVICE_UUID);

  // TX
  pTxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  // RX
  pRxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE
  );
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  pServer->getAdvertising()->start();
  
  Serial.println("Waiting for BluefruitConnect -> ESP32 GoPro -> Connect");
  while (deviceConnected == false)
  {
    Serial.print(".");
    digitalWrite(2, HIGH);
    delay(250);
    digitalWrite(2, LOW);
    delay(250);
    digitalWrite(2, HIGH);
    delay(500);
    digitalWrite(2, LOW);
    delay(500);
  }
  Serial.println(" -> UART (within 5 seconds;)");
  delay(5000);
//  HelpDisplay();
  
  Serial.println("Starting BLE Client...");
  //BLEDevice::init("");

  keepAliveTicker = millis();

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);

  Serial.print("Scan for GoPro: ");
  while (foundBLEService == false)
  {
    Serial.print('*');
    delay(1000);
  }
  Serial.println("");
  
  if (connectToBLEServer() == false)
  {
    Serial.println("FAILED to find GoPro!");
    
    return;
  }    

    Serial.println("Connected to GoPro BLE Server.");
    delay(1000);

    SetVideoMode();  

} // End of setup.


// This is the Arduino main loop function.
void loop() 
{
  char in = 0;
  int iRet;
  bool bGoProCommand = false;

  if (espSerialPhone.available() > 0)
  {
    char sBuff[2] = " ";
    sBuff[0] = espSerialPhone.read();

    if (sBuff[0] == 0x1B)
      bGoProCommand = true;
    else
      ble_send(sBuff);
  }

  if (bGoProCommand)
  {
    while (espSerialPhone.available() == 0)
    ;
    
    if (espSerialPhone.available() > 0)
    {
      in = espSerialPhone.read();
#if DEBUG_OUTPUT
  {
  Serial.print("GoProCommand = ESC,");
  Serial.print(in);
  Serial.println("");
  }
#endif
  
  
  //////////////////////////////////////////////////////////////////////
      switch (in)
      {
      default:
        break;

      // Take a picture or start a video
      case 'A':
        bluefruitconnectstartrecording = true;
        bluefruitconnectstoprecording = false;
        //SendString_ble("00 Start Recording\n");
        espSerialPhone.print('1');
        break;
    
      // Stop the video
      case 'S':
        bluefruitconnectstartrecording = false;
        bluefruitconnectstoprecording = true;
        //SendString_ble("Stop Recording\n");
//        espSerialPhone.print('1');
// Note: reply with '1' after video is stopped and written to SD card
        break;

      // Set Video mode
      case 'V':
        SetVideoMode();  
        espSerialPhone.print('1');
        break;

      // Set Photo mode
      case 'P':
        SetPhotoMode();
        espSerialPhone.print('1');
        break;
                
      // Get current time
      case '2':
      {
        struct tm timeinfo;
        if(!getLocalTime(&timeinfo)){
          Serial.println("Failed to obtain time");
          return;
        }
        espSerialPhone.print(&timeinfo, "%Y-%m-%d %H:%M:%S");
        //GetLocalTime();
        //espSerialPhone.print(timeBuffer);
        break;
      }    

      case '3':
      {
        daylightOffset_sec = 3600;
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        printLocalTime();
        espSerialPhone.print('1');
        break;
      }

      case '4':
      {
        daylightOffset_sec = 0;
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        printLocalTime();
        espSerialPhone.print('1');
        break;
      }

      // Open the connection
      case '1':
        espSerialPhone.print('1');
        break;
    
      // Close the connection
      case '0':
        espSerialPhone.print('1');
        break;
      }
//////////////////////////////////////////////////////////////////////
  }

}

  if (deviceConnected == false)
  {
    Serial.println("Waiting for BluefruitConnect -> ESP32 (Connect) -> UART");
    delay(5000);
    return;
  }
  
  ticker++;
  
  if (connectedBLE == true)
  {
    if (millis() > (keepAliveTicker + 120000))
    {
      KeepAlive();
      keepAliveTicker = millis();
    }
  }
  else
  {
    if ((ticker % 3) == 0)
    {
      if (onOffLED == false)
      {
        onOffLED = true;
        digitalWrite(2, HIGH);
      }
      else
      {
        onOffLED = false;
        digitalWrite(2, LOW);
      }
    }
    delay(500); // Delay a second between loops.
    return;
  }
    
  if (bluefruitconnectstoprecording)
  {
    bluefruitconnectstartrecording = false;
    bluefruitconnectstoprecording = false;
    StopVideo();
    espSerialPhone.print('1');
    keepAliveTicker = millis();
    return;        
  }
  
  if (bluefruitconnectstartrecording)
  {
    digitalWrite(2, HIGH);
  }
  else
  {    
    digitalWrite(2, LOW);
    return;
  }
  /*
    If the flag "foundBLEService" is true then it has found the desired
    BLE Server with which we wish to connect. Once we are
    connected it will set the connectedBLE flag to be true.
  */

  if (connectedBLE == true)
  {
      Serial.println("Taking video...");

      StartVideo();
      bluefruitconnectstartrecording = false;
  } 
  else 
  if (doScan) 
  {
    BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
  }
  delay(500); // Delay a second between loops.

} // End of loop

void SetVideoMode()
{
  uint8_t array[7];

  Serial.println("Set Video Mode and wait for response...");
  waitForCommandNotify = false;
  array[0] = 0x04;
  array[1] = 0x3e;
  array[2] = 0x02;
  array[3] = 0x03;
  array[4] = 0xe8;
  array[5] = 0x00;
  array[6] = 0x00;
  pCommandCharacteristic->writeValue(array, 5);
  while (waitForCommandNotify == false)
  {
    Serial.print("!");
    delay(500);
  }
}

void SetPhotoMode()
{
  uint8_t array[7];

  Serial.println("Set Photo Mode and wait for response...");
  waitForCommandNotify = false;
  array[0] = 0x04;
  array[1] = 0x3e;
  array[2] = 0x02;
  array[3] = 0x03;
  array[4] = 0xe9;
  array[5] = 0x00;
  array[6] = 0x00;
  pCommandCharacteristic->writeValue(array, 5);
  while (waitForCommandNotify == false)
  {
    Serial.print("!");
    delay(500);
  }  
}

void TakePhoto()
{
  uint8_t array[7];

  Serial.println("Shutter ON in Photo Mode and wait for response...");
  waitForCommandNotify = false;
  array[0] = 0x03;
  array[1] = 0x01;
  array[2] = 0x01;
  array[3] = 0x01;
  array[4] = 0x00;
  array[5] = 0x00;
  array[6] = 0x00;
  pCommandCharacteristic->writeValue(array, 4);
  while (waitForCommandNotify == false)
  {
    Serial.print("!");
    delay(500);
  }

  Serial.println("Shutter OFF in Photo Mode and wait for response...");
  waitForCommandNotify = false;
  array[0] = 0x03;
  array[1] = 0x01;
  array[2] = 0x01;
  array[3] = 0x00;
  array[4] = 0x00;
  array[5] = 0x00;
  array[6] = 0x00;
  pCommandCharacteristic->writeValue(array, 4);
  while (waitForCommandNotify == false)
  {
    Serial.print("!");
    delay(500);
  }

  while (true)
  {
    Serial.println("Wait while busy writing photo...");
    waitForQueryNotify = false;
    array[0] = 0x02;
    array[1] = 0x13;
    array[2] = 0x08;
    array[3] = 0x00;
    array[4] = 0x00;
    array[5] = 0x00;
    array[6] = 0x00;
    pQueryCharacteristic->writeValue(array, 3);
    while (waitForQueryNotify == false)
    {
      Serial.print("!");
      delay(500);
    }
    if (notifyQueryData[5] == 0)
      break;
  }
}

void StartVideo()
{
  uint8_t array[7];

  Serial.println("Shutter ON and wait for response...");
  waitForCommandNotify = false;
  array[0] = 0x03;
  array[1] = 0x01;
  array[2] = 0x01;
  array[3] = 0x01;
  array[4] = 0x00;
  array[5] = 0x00;
  array[6] = 0x00;
  pCommandCharacteristic->writeValue(array, 4);
  while (waitForCommandNotify == false)
  {
    Serial.print("!");
    delay(500);
  }  
}

void StopVideo()
{
  uint8_t array[7];

  Serial.println("Shutter OFF and wait for response...");
  waitForCommandNotify = false;
  array[0] = 0x03;
  array[1] = 0x01;
  array[2] = 0x01;
  array[3] = 0x00;
  array[4] = 0x00;
  array[5] = 0x00;
  array[6] = 0x00;
  pCommandCharacteristic->writeValue(array, 4);
  while (waitForCommandNotify == false)
  {
    Serial.print("!");
    delay(500);
  }

  while (true)
  {
    Serial.println("Wait while busy writing video...");
    waitForQueryNotify = false;
    array[0] = 0x02;
    array[1] = 0x13;
    array[2] = 0x08;
    array[3] = 0x00;
    array[4] = 0x00;
    array[5] = 0x00;
    array[6] = 0x00;
    pQueryCharacteristic->writeValue(array, 3);
    while (waitForQueryNotify == false)
    {
      Serial.print("!");
      delay(500);
    }
    if (notifyQueryData[5] == 0)
      break;
  } 
}

void GetDateTime()
{
  uint8_t array[7];

#if 0  
  Serial.println("Register for status updates");
  waitForQueryNotify = false;
  array[0] = 0x02;
  array[1] = 0x53;
  array[2] = 0x28;
  array[3] = 0x00;
  array[4] = 0x00;
  array[5] = 0x00;
  array[6] = 0x00;
  pQueryCharacteristic->writeValue(array, 3);
  while (waitForQueryNotify == false)
  {
    Serial.print("!");
    delay(500);
  }
#endif

  Serial.println("Get GoPro Date Time");
  waitForQueryNotify = false;
  array[0] = 0x02;
//  array[1] = 0x13;
  array[1] = 0x93;
  array[2] = 0x28;
  array[3] = 0x00;
  array[4] = 0x00;
  array[5] = 0x00;
  array[6] = 0x00;
  pQueryCharacteristic->writeValue(array, 3);
  while (waitForQueryNotify == false)
  {
    Serial.print("!");
    delay(500);
  }

#if 0
  Serial.println("Unregister for status updates");
  waitForQueryNotify = false;
  array[0] = 0x02;
  array[1] = 0x73;
  array[2] = 0x28;
  array[3] = 0x00;
  array[4] = 0x00;
  array[5] = 0x00;
  array[6] = 0x00;
  pQueryCharacteristic->writeValue(array, 3);
  while (waitForQueryNotify == false)
  {
    Serial.print("!");
    delay(500);
  }
#endif


}

void KeepAlive()
{
  uint8_t array[7];

  Serial.println("Keep Alive");
  waitForSettingsNotify = false;
  array[0] = 0x03;
  array[1] = 0x5B;
  array[2] = 0x01;
  array[3] = 0x42;
  array[4] = 0x00;
  array[5] = 0x00;
  array[6] = 0x00;
  pSettingsCharacteristic->writeValue(array, 4);
  while (waitForSettingsNotify == false)
  {
    Serial.print("!");
    delay(500);
  }
}

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%Y-%m-%d %H:%M:%S");
}

void SendString_ble(char *str)
{
//  ble_send("AT+BLEUARTTX=");
  ble_send(str);
  ble_send("\n");
  // check response stastus
  //if (! ble.waitForOK() ) {
    //Serial.println(F("Failed to send?"));
  //}
}
