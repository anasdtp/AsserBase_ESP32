//----------------------------------------------------------------------Bibliotheques
#include <Arduino.h>
#include "CRAC_utility.h"
#include <CAN/src/CAN.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
//----------------------------------------------------------------------Variables

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;
bool canAvailable = false;
bool newCan = false, BtAvailable = false;
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "69a68877-c627-42d1-a087-6a1307f83616" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


typedef struct CANMessage {
  bool extented = false;
  bool RTR = false;
  unsigned int ID = 0;
  char ln = 0;
  unsigned char dt[8] = {0};
} CANMessage;
CANMessage myData;//data received by BT to write on CAN
CANMessage DATAtoSend;//data received by CAN to send on BT
CANMessage DATAtoControl;//data received by CAN to control the robot
CANMessage DATArobot;//DATA that the robot will write on CAN
//----------------------------------------------------------------------prototypes fonctions BLE et CAN

char vitesse_danger = 0, Stop_Danger = 0, asser_actif = 1, attente = 0, mode_xyt = 0,
                finMvtElem = 0, finRecalage = 0, finRayonCourbure = 0,finRayonCourbureClo = 0, finXYT = 0,  Fin_Match = 0, Message_Fin_Mouvement = 0, explosionErreur = 0; 
int nb_ordres = 0;
// Tout plein de flags
short           etat_automate = 0, etat_automate_depl = 0, new_message = 0,
                xytheta_sens, next_move_xyt = 0, next_move = 0, i, stop = 0, stop_receive = 0, param_xytheta[3],
                etat_automate_xytheta = 0, ralentare = 0;