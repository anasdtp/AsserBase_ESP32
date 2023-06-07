//----------------------------------------------------------------------Bibliotheques
#include <Arduino.h>
#include "CRAC_utility.h"
#include <CAN.h>
//----------------------------------------------------------------------Variables
#define SIZE_FIFO 32

typedef struct CANMessage {
  bool extented = false;
  bool RTR = false;
  unsigned int ID = 0;
  char ln = 0;
  unsigned char dt[8] = {0};
} CANMessage;
CANMessage myData;//data received by BT to write on CAN
CANMessage DATAtoSend;//data received by CAN to send on BT
CANMessage rxMsg[SIZE_FIFO];//data received by CAN to control the robot
CANMessage DATArobot;//DATA that the robot will write on CAN

unsigned char FIFO_ecriture = 0;

char vitesse_danger = 0, Stop_Danger = 0, asser_actif = 1, attente = 0, mode_xyt = 0,
                finMvtElem = 0, finRecalage = 0, finRayonCourbure = 0,finRayonCourbureClo = 0, finXYT = 0,  Fin_Match = 0, Message_Fin_Mouvement = 0, explosionErreur = 0; 
int nb_ordres = 0;
// Tout plein de flags
short           etat_automate = 0, etat_automate_depl = 0, new_message = 0,
                xytheta_sens, next_move_xyt = 0, next_move = 0, i, stop = 0, stop_receive = 0, param_xytheta[3],
                etat_automate_xytheta = 0, ralentare = 0;