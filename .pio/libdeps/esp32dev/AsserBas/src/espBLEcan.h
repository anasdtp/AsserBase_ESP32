//----------------------------------------------------------------------Bibliotheques
#include <Arduino.h>
#include <CAN.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
//----------------------------------------------------------------------Variables



#define bleServerName "ESP32EB"
#define peerName "ESP32EA";
//SLAVE-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_


/* UUID's of the service, characteristic that we want to read*/
// BLE Service
static BLEUUID serviceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID      rxUUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID      txUUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_SLAVE

//Address of the peripheral device. Address will be found during scanning...
static BLEAddress *pServerAddress;

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* prxRemoteCharacteristic;//Characteristicd that we want to read
static BLERemoteCharacteristic* ptxRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;
//Activate notify
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t notificationOff[] = {0x0, 0x0};

char* contenuBt;
//Flags to check whether new CAN and humidity readings are available
boolean newCan = false;
bool canAvailable = false;

typedef struct CANMessage {
  bool extented = false;
  bool RTR = false;
  unsigned int ID;
  char ln;
  unsigned char dt[8];

  char BLEID = 0x42;//Nom du device BLE 0x41=A, 0x42=B
} CANMessage;

