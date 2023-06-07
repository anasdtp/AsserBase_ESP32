#include "espCan.h"

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

void remplirStruct(CANMessage &theDATA, int idf, char lenf, char dt0f, char dt1f, char dt2f, char dt3f, char dt4f, char dt5f, char dt6f, char dt7f){
  theDATA.RTR = false;
  if(idf>0x7FF){theDATA.extented = true;}
  else{theDATA.extented = false;}
  theDATA.ID = idf;
  theDATA.ln = lenf;
  theDATA.dt[0] = dt0f;
  theDATA.dt[1] = dt1f;
  theDATA.dt[2] = dt2f;
  theDATA.dt[3] = dt3f;
  theDATA.dt[4] = dt4f;
  theDATA.dt[5] = dt5f;
  theDATA.dt[6] = dt6f;
  theDATA.dt[7] = dt7f;
}

void writeStructInCAN(const CANMessage &theDATA){
  //Serial.print("Sending ");
  if(theDATA.extented){
    CAN.beginExtendedPacket(theDATA.ID, theDATA.ln, theDATA.RTR);
    Serial.print("extended ");
  }
  else{CAN.beginPacket(theDATA.ID, theDATA.ln, theDATA.RTR);}
  //Serial.print("packet on CAN...");
  if(!theDATA.RTR){CAN.write(theDATA.dt, theDATA.ln);}
  
  CAN.endPacket();
  /*Serial.print(" ID : 0x");
  Serial.print(theDATA.ID, HEX);
  //Serial.println(" done");
  //Serial.println();*/
}
