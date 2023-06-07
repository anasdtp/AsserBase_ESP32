//----------------------------------------------------------------------Bibliotheques
#include <Arduino.h>
#include <CRAC_utility.h>
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
extern CANMessage myData;//data received by BT to write on CAN
extern CANMessage DATAtoSend;//data received by CAN to send on BT
extern CANMessage rxMsg[SIZE_FIFO];//data received by CAN to control the robot
extern CANMessage DATArobot;//DATA that the robot will write on CAN

extern unsigned char FIFO_ecriture;

extern char vitesse_danger, Stop_Danger, asser_actif, attente, mode_xyt,
                finMvtElem, finRecalage, finRayonCourbure,finRayonCourbureClo, finXYT,  Fin_Match, Message_Fin_Mouvement, explosionErreur; 
extern int nb_ordres;
// Tout plein de flags
extern short           etat_automate, etat_automate_depl, new_message,
                xytheta_sens, next_move_xyt, next_move, i, stop, stop_receive, param_xytheta[3],
                etat_automate_xytheta, ralentare;



//prototypes fonctions CAN :
void setupCAN();

void canReadData(int packetSize);
void canReadExtRtr();

void remplirStruct(CANMessage &theDATA, int id, char len, char dt0, char dt1, char dt2, char dt3, char dt4, char dt5, char dt6, char dt7);
void remplirStruct2x4Bytes(uint32_t id, void *pdata1, void *pdata2);

void writeStructInCAN(const CANMessage &theDATA);
void CANenvoiMsg1x8Bytes(uint32_t id, void *pdata);
void CANenvoiMsg2x4Bytes(uint32_t id, void *pdata1, void *pdata2);
void CANenvoiMsg3x2Bytes(uint32_t id, int16_t data1, int16_t data2, int16_t data3);

void printCANMsg(CANMessage& msg);