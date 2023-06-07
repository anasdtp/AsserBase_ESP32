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

bool onPrendsEnCompte(uint16_t ID);




void setupCAN(){
  while (!Serial);
  //Serial.println("Base Roulante de Anas Le bg/dg");
  // start the CAN bus at 1000 kbps
  if (!CAN.begin(1000E3)) { //ici, nous avons modifié la bibliotheque CAN pour qu'elle soit compatible avec l'ESP32-E
    //Serial.println("Starting CAN failed!");
    while (1);
  }
  CAN.onReceive(canReadData); //init CAN callback function
}


void canReadData(int packetSize){
  remplirStruct(rxMsg[FIFO_ecriture], 0,0,0,0,0,0,0,0,0,0); //A tester si ça ne surcharge pas
  rxMsg[FIFO_ecriture].ID = CAN.packetId();
  if(!onPrendsEnCompte(rxMsg[FIFO_ecriture].ID)){return;}//A regarder si rajouter ça ça ne surcharge pas l'interruption
  rxMsg[FIFO_ecriture].ln = CAN.packetDlc();
  //Serial.printf("Received CAN, ID : 0x%.3X ; len : %d\n", rxMsg[FIFO_ecriture].ID, rxMsg[FIFO_ecriture].ln);
  // only print packet rxMsg[FIFO_ecriture].dt for non-RTR packets
  int i = 0;
  while (CAN.available())
  {
    rxMsg[FIFO_ecriture].dt[i]=CAN.read();
    i++;
  }
//   canAvailable = true;
    FIFO_ecriture=(FIFO_ecriture+1)%SIZE_FIFO;
}

void canReadExtRtr(){//On n'execute plus cette fonction pour compléter la msg CAN car on part du principe qu'on n'utilise pas de ID extented et de msg rtr
  if (CAN.packetExtended()){
      //Serial.print("extended ");
      rxMsg[FIFO_ecriture].extented = true;
  }else{rxMsg[FIFO_ecriture].extented = false;}
  if (CAN.packetRtr()){
      // Remote transmission request, packet contains no rxMsg[FIFO_ecriture].dt
      //Serial.print("RTR ");
      rxMsg[FIFO_ecriture].RTR = true;
  }
  else{rxMsg[FIFO_ecriture].RTR = false;}
  
  //printCANMsg(rxMsg[FIFO_ecriture]);
//   FIFO_ecriture=(FIFO_ecriture+1)%SIZE_FIFO;
}




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
void remplirStruct2x4Bytes(uint32_t id, void *pdata1, void *pdata2){
    DATArobot.RTR = false;
    if(id>0x7FF){DATArobot.extented = true;}
    else{DATArobot.extented = false;}
    DATArobot.ID = id;
    DATArobot.ln = 8;
    memcpy(&DATArobot.dt, pdata1, 4);
    memcpy(&(DATArobot.dt[4]), pdata2, 4);
    
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

void CANenvoiMsg1x8Bytes(uint32_t id, void *pdata)
{
    DATArobot.RTR = false;
    if(id>0x7FF){DATArobot.extented = true;}
    else{DATArobot.extented = false;}
    DATArobot.ID = id;
    DATArobot.ln = 8;
    memcpy(&DATArobot.dt, pdata, 8);
    writeStructInCAN(DATArobot);          
}
void CANenvoiMsg2x4Bytes(uint32_t id, void *pdata1, void *pdata2)
{
    DATArobot.RTR = false;
    if(id>0x7FF){DATArobot.extented = true;}
    else{DATArobot.extented = false;}
    DATArobot.ID = id;
    DATArobot.ln = 8;
    memcpy(&DATArobot.dt, pdata1, 4);
    memcpy(&(DATArobot.dt[4]), pdata2, 4);
    writeStructInCAN(DATArobot);     
}

void CANenvoiMsg3x2Bytes(uint32_t id, int16_t data1, int16_t data2, int16_t data3){
    DATArobot.RTR = false;
    if(id>0x7FF){DATArobot.extented = true;}
    else{DATArobot.extented = false;}
    DATArobot.ID = id;
    DATArobot.ln = 6;
    DATArobot.dt[0] = data1 & 0xFF;
    DATArobot.dt[1] = (data1 >> 8) & 0xFF;
    DATArobot.dt[2] = data2 & 0xFF;
    DATArobot.dt[3] = (data2 >> 8) & 0xFF;
    DATArobot.dt[4] = data3 & 0xFF;
    DATArobot.dt[5] = (data3 >> 8) & 0xFF;
    writeStructInCAN(DATArobot);
}

void printCANMsg(CANMessage& msg) {
    printf("  ID      = 0x%.3x\n", msg.ID);
    printf("  extented    = %d\n", msg.extented);
    printf("  format rtr = %d\n", msg.RTR);
    printf("  Length  = %d\n", msg.ln);
    printf("  Data    = 0x");            
    for(int i = 0 ; i < msg.ln ; i++){printf(" %.2X", msg.dt[i]);}
    printf("\n");
 }




bool onPrendsEnCompte(uint16_t ID){
    switch (ID)
    {
            case ESP32_RESTART:
                return true;
                break;
            case ASSERVISSEMENT_REQUETE_PID:
                return true;
                break;
            case ASSERVISSEMENT_CONFIG_KPP_DROITE:
                return true;
                break;
            case ASSERVISSEMENT_CONFIG_KPI_DROITE:
                return true;
                break;
            case ASSERVISSEMENT_CONFIG_KPD_DROITE:
                return true;
                break;
                
            case ASSERVISSEMENT_CONFIG_KPP_GAUCHE:
                return true;
                break;
            case ASSERVISSEMENT_CONFIG_KPI_GAUCHE :
                return true;
                break;
            case ASSERVISSEMENT_CONFIG_KPD_GAUCHE :
                return true;
                break;
                
            case ASSERVISSEMENT_CONFIG_KPP:{
                return true;
                }
                break;
            case ASSERVISSEMENT_CONFIG_KPI :
                return true;
                break;
            case ASSERVISSEMENT_CONFIG_KPD :
                return true;
                break;

            case ASSERVISSEMENT_CONFIG_PERIMETRE_ROUE_CODEUSE :
                return true;
                break;
            case ASSERVISSEMENT_CONFIG_LARGEUR_ROBOT :
                return true;
                break;
            case ECRAN_CHOICE_COLOR :
                return true;
                break;
                
            case ASSERVISSEMENT_ENABLE :
                return true;
            break;
                
            case ASSERVISSEMENT_DECEL :
                return true;
            break;
            case ASSERVISSEMENT_XYT :
                return true;
            break;
            case ASSERVISSEMENT_COURBURE:
                return true;
            break;  
            case ASSERVISSEMENT_CONFIG:
                return true;
            break;
            case ASSERVISSEMENT_ROTATION:
                return true;
            break;  
            case ASSERVISSEMENT_RECALAGE:
                return true;
            break;
            case ASSERVISSEMENT_BEZIER:
                return true;
            break;  
            case ODOMETRIE_SMALL_POSITION:
                return true;
            break;
            case ODOMETRIE_SMALL_VITESSE:
                return true;
            break;
            case ODOMETRIE_BIG_POSITION:
                return true;
            break;
            case ODOMETRIE_BIG_VITESSE:
                return true;
            break;
            case GLOBAL_GAME_END:
                return true;
            break;
            case ASSERVISSEMENT_STOP:
                return true;
            break;  
            case CHECK_MOTEUR:
                return true;
            break;

//---------------------------------------------Qt
            case ASSERVISSEMENT_CONFIG_KPP_Qt:
                return true;
                break;
            case ASSERVISSEMENT_CONFIG_KPI_Qt :
                return true;
                break;
            case ASSERVISSEMENT_CONFIG_KPD_Qt :
                return true;
                break;
            case ASSERVISSEMENT_CONFIG_LARGEUR_ROBOT_Qt :
                return true;
                break;
            case ASSERVISSEMENT_CONFIG_PERIMETRE_ROUE_CODEUSE_Qt :
                return true;
                break;
            case IDCAN_POS_XY_OBJET:{
                return true;
            }break;
            
            default :
            return false;
            break;
    }

    return false;
  
}

