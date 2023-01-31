//----------------------------------------------------------------------Bibliotheques
#include <Arduino.h>
#include "espBLEcan.h"
#include "CRAC_utility.h"
#include "ident_crac.h"
#include "buffer_circulaire.h"
#include "clotho.h"

//----------------------------------------------------------------------Variables


bool set = false;
int inApin_MOTD = 16; // INA2 checked
int inApin_MOTG = 26; // INA1 checked
int inBpin_MOTD = 15; // INB2 checked
int inBpin_MOTG = 25; // INB1 checked
  // PWM moteur

int joyMot = 500;
double erreur = 3;
/****************************************************************************************/
/*                           Definition et affectation des variables                   */
/*            Par souci de simplicité, on utilise beaucoup de variables globales        */
/****************************************************************************************/
double          consigne_pos, consigne_vit,                 // Consignes de position et de vitesse dans les mouvements
                VMAX, AMAX, AMAX_CLO, DMAX,                 // Valeurs maximales d'accéleration, décélération et vitesse
                Odo_x, Odo_y, Odo_theta, Odo_val_pos_D, Odo_val_pos_G, Odo_last_val_pos_D, Odo_last_val_pos_G,  // Variables de positions utilisées pour l'odométrie
                roue_drt_init, roue_gch_init,               // Valeur des compteurs (!= 0) quand on commence un nouveau mouvement
                global_ta_stop=0,global_decel_stop=0,
                old_posD=0,old_posG=0;
                
unsigned short cpt = 0,cpt_arret=0;
int cpt_ordre = 0;
                
                
volatile uint16_t          mscount = 0,      // Compteur utilisé pour envoyer échantillonner les positions et faire l'asservissement
                         mscount1 = 0,     // Compteur utilisé pour envoyer échantillonner les positions et faire l'asservissement
                         mscount2 = 0;     // Compteur utilisé pour envoyer la trame CAN d'odométrie
double consigne_posG = 0, consigne_posD = 0;
float distanceG = 0, distanceD = 0;
char flagDebutBezier = 0;
int nbValeurs = 0;
int nbprint = 0;

double Kp =8    , Ki =0.0, Kd =35.0;
                
/****************************************************************************************/
//Sinon, détails : les angles sont exprimés en dixièmes de degrés quand il faut faire des calculs, ou quand ils faut les transmettre en CAN
//les distances, pareil, c'est de millimètres
//Par contre, pour les fonctions Ligne_droite, Rotation, et asser_pos c'est exprimé en ticks d'encodeur
/****************************************************************************************/
                             
/****************************************************************************************/
/*                               Deplacement a effectuer                                */
/*                Liste des ordres de déplacement à effectuer à la suite                */
/*                    (PAS IMPLEMENTE, OU ALORS CA MARCHE TRES MAL)                     */
/****************************************************************************************/ 
struct Ordre_deplacement liste;
clothoStruc maClotho;
//----------------------------------------------------------------------Timer
static char idTimer = 0; //le numéro du Timer de 0 à 3
static int prescaler = 8000; // la valeur du diviseur de temps
bool flag = true; //vrai pour compter sur le front montant, faux pour compter sur le front descendant
int totalInterrupts = 0;   // compte le nombre de declenchement de l alarme
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
void init_Timer();
void onTime();//prototype de la fonction s'exécutent à chaque interruptions
//----------------------------------------------------------------------prototypes fonctions 
//Fonctions principales : 
void calcul(void);
void CANloop();
void Odometrie(void);

//Fonctions Asservissement : 
void init_coef();
void Mouvement_Elementaire(long pcons, short vmax, short amax, short dmax, char mvt);
void Rayon_De_Courbure(short rayon, short theta, short vmax, short amax, short sens, short dmax);
void trait_Mouvement_Elementaire_Gene(struct Ordre_deplacement* monDpl);
void trait_Rayon_De_Courbure_Clotho(struct Ordre_deplacement* monDpl);
void Mouvement_Elementaire_Gene(struct Ordre_deplacement monDpl);
void Rayon_De_Courbure_Clotho(struct Ordre_deplacement monDpl);
void X_Y_Theta(long px, long py, long ptheta, long sens, short vmax, short amax);
void Recalage(int pcons, short vmax, short amax, short dir, short nv_val);
int Courbe_bezier(double distanceG, double distanceD);

//Fonctions Moteurs :
void Moteur_Init();
void setupPWM(int PWMpin, int PWMChannel);

//prototypes fonctions BLE et CAN :
void setupCAN();
void writeStructInCAN(const CANMessage &theDATA);
void canReadData(int packetSize);
void canReadExtRtr();
void CANloop();
void masterBTConnect(std::string name);
bool connectToServer(BLEAddress pAddress);
void readDATA(std::string contenuBT, CANMessage &theDATA);
void printCANMsg(CANMessage& msg);
void remplirStruct(int id, char len, char dt0, char dt1, char dt2, char dt3, char dt4, char dt5, char dt6, char dt7, CANMessage &theDATA);
void CANenvoiMsg1x8Bytes(uint32_t id, void *pdata);
void CANenvoiMsg2x4Bytes(uint32_t id, void *pdata1, void *pdata2);
void remplirStruct2x4Bytes(uint32_t id, void *pdata1, void *pdata2);
void CANenvoiMsg3x2Bytes(uint32_t id, int16_t data1, int16_t data2, int16_t data3);
void BtActualise();
//----------------------------------------------------------------------callback fonctions
class MyServerCallbacks : public BLEServerCallbacks {
	void onConnect(BLEServer* pServer) {
		deviceConnected = true;
    BLEDevice::startAdvertising();
	}
	;

	void onDisconnect(BLEServer* pServer) {
		deviceConnected = false;
	}
}
;

class MyCallbacks : public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic *pCharacteristic) {
		std::string rxValue = pCharacteristic->getValue();
    readDATA(rxValue, DATAtoControl);
    newCan = true;
	}
};
//----------------------------------------------------------------------SETUP
void setup() {
  Serial.begin(921600);
  init_coef();
  setupCAN();
  //Init BLE device
  masterBTConnect("BaseRoulante");
  Serial.printf("fin ble init\n"); //Ne pas rajouter de Serial.printx dans le loop 
                                  //sinon cela augmente le temps de loop on risque d'avoir un TE faux
  Encodeur_Init();
  Serial.printf("fin encodeur init\n");
  Moteur_Init();
  Serial.printf("fin moteur init\n");
  AsserInitCoefs(Kp, Ki, Kd);
  Asser_Init();
  init_Timer();
  Serial.printf("fin init Timer\n");
  remplirStruct(ALIVE_MOTEUR,0,0,0,0,0,0,0,0,0, DATArobot);
  writeStructInCAN(DATArobot);
  Serial.printf("envoie CAN ALIVE_MOTEUR\n");
  Serial.println("fin setup\n");
  
  /*liste.type = TYPE_DEPLACEMENT_LIGNE_DROITE;//test
  liste.distance = 1000;*/
}
//----------------------------------------------------------------------loop
void loop() {
  CANloop();
  calcul();
  Odometrie();
  
  //test du moteur droit ainsi que de son asservissement par la roue encodeuse droitey
  
  if (mscount >= (TE_100US)) 
  {   
    Serial.println("erreur temp calcul");
    Serial.println(mscount);
    remplirStruct(ERREUR_TEMP_CALCUL,2, mscount,TE_100US,0,0,0,0,0,0, DATArobot);
    writeStructInCAN(DATArobot);
    
    if (deviceConnected){
        pTxCharacteristic->setValue((uint8_t *)&DATArobot, sizeof(DATArobot));
		pTxCharacteristic->notify();
    //Serial.println("Sending via BT...");
    }else{if(!nbprint){Serial.println("The device is not connected"); nbprint ++;}}

    
  }
  else 
  {
    while (mscount<(TE_100US)){
        
    }
    
  }

  //digitalWrite(27, set);//pour mesurer le temps de boucle avec l'oscilloscope
  //set = !set;

  BtActualise();
  mscount = 0;    

  

     
}
//----------------------------------------------------------------------fonctions
/***************************************************************************************
 NOM : calcul                                                              
 ARGUMENT : aucun                                        
 RETOUR : rien                                                                        
 DESCRIPTIF :   Fonction principale appelé periodiquement, qui gère l'ensemble
                Se base sur : struct Ordre_deplacement liste;
                liste d'ordre remplie par l'ISR CAN
                struct Ordre_deplacement
                {
                    char type, enchainement;
                    short vmax, amax;
                    long distance, recalage, val_recalage;
                    long angle;
                    short x, y, theta;
                    signed char sens;
                    short rayon, vit_ray, theta_ray;
                };
***************************************************************************************/
void calcul(void){//fait!!
    static int cpt_stop = 0;
    static char etat_prec = TYPE_DEPLACEMENT_IMMOBILE, etat_automate_depl_prec = INITIALISATION;
    if(stop_receive!=1)
    {
        #if F_DBUG_ETAT
        if(etat_prec != liste.type)
        {
            etat_prec = liste.type;
            
            Serial.println("ID_DBUG_ETAT");
            remplirStruct(ID_DBUG_ETAT, 1, etat_prec, 0,0,0,0,0,0,0, DATArobot);
            writeStructInCAN(DATArobot);                             //CAN
            if (deviceConnected){
                pTxCharacteristic->setValue((uint8_t *)&DATArobot, sizeof(DATArobot));
		        pTxCharacteristic->notify();
    //Serial.println("Sending via BT...");
        }else{if(!nbprint){Serial.println("The device is not connected"); nbprint ++;}}
            
            #if F_DBUG_ETAT_DPL
            remplirStruct(ID_DBUG_ETAT_DPL, 1, etat_automate_depl, 0,0,0,0,0,0,0, DATArobot);
            writeStructInCAN(DATArobot);                             //CAN
            if (deviceConnected){
                pTxCharacteristic->setValue((uint8_t *)&DATArobot, sizeof(DATArobot));
		        pTxCharacteristic->notify();
    //Serial.println("Sending via BT...");
        }else{if(!nbprint){Serial.println("The device is not connected"); nbprint ++;}}
            #endif
        }
        #endif
        
        #if F_DBUG_ETAT_DPL
        if(etat_automate_depl_prec != etat_automate_depl && etat_automate_depl != 8)
        {
            etat_automate_depl_prec = etat_automate_depl;
            //Serial.println("ID_DBUG_ETAT_DPL");
            remplirStruct(ID_DBUG_ETAT_DPL, 1, etat_automate_depl_prec, 0,0,0,0,0,0,0, DATArobot);
            writeStructInCAN(DATArobot);                             //CAN           
        }
       #endif
       
        if (Fin_Match){
            liste.type = (TYPE_END_GAME);
            //Serial.println("INSTRUCTION_END_MOTEUR");
            //On prévient qu'on s'est arrêté
            // le dlc original était de 2 avec un 0 en second octet
            remplirStruct(INSTRUCTION_END_MOTEUR, 1, 0x04, 0,0,0,0,0,0,0, DATArobot);
            writeStructInCAN(DATArobot);                             //CAN
            Fin_Match = 0;
        }
        else if (!asser_actif)
        {
            liste.type = (TYPE_ASSERVISSEMENT_DESACTIVE);
        }
          
        double cmdD, cmdG;
        switch(liste.type)
        {
            case (TYPE_END_GAME) :{
            Arret();
            break;
            }
            case (TYPE_DEPLACEMENT_IMMOBILE):{
            cmdD = Asser_Pos_MotD(roue_drt_init);
            cmdG = Asser_Pos_MotG(roue_gch_init);
            write_PWMD(cmdD);
            write_PWMG(cmdG);   
            break;
            }            
            case (TYPE_DEPLACEMENT_LIGNE_DROITE):{
            Message_Fin_Mouvement = ASSERVISSEMENT_RECALAGE;
            Mouvement_Elementaire(liste.distance, VMAX, AMAX, DMAX, MOUVEMENT_LIGNE_DROITE); 
            if (finMvtElem)
            {
                liste.type = (TYPE_MOUVEMENT_SUIVANT);
                finMvtElem = 0;
            }
/*            else if (stop_receive)
            {
                liste.type = TYPE_INIT_STOP;
    //            write_PWMD(0);
    //            write_PWMG(0);
            }
*/            break;
              }
            case (TYPE_DEPLACEMENT_ROTATION):{
            Message_Fin_Mouvement = ASSERVISSEMENT_ROTATION;
            Mouvement_Elementaire(liste.angle, VMAX/3, AMAX, DMAX, MOUVEMENT_ROTATION);
            if (finMvtElem){
                liste.type = (TYPE_MOUVEMENT_SUIVANT);
                finMvtElem = 0;
            }
            /*else if (stop_receive){
                liste.type = TYPE_INIT_STOP;
                write_PWMD(0);
                write_PWMG(0);
            }*/
            break;
            }
            case (TYPE_DEPLACEMENT_X_Y_THETA):{
            Message_Fin_Mouvement = ASSERVISSEMENT_XYT;
            X_Y_Theta(liste.x, liste.y, liste.theta, liste.sens, VMAX, AMAX); 
            if (finXYT){
                liste.type = (TYPE_MOUVEMENT_SUIVANT);
                finXYT = 0;
            }    
/*            else if (stop_receive){
                liste.type = TYPE_INIT_STOP;
                //Arret_Brutal();
            }
*/            break;
              }
            case (TYPE_DEPLACEMENT_RAYON_COURBURE) :{
            Message_Fin_Mouvement = ASSERVISSEMENT_COURBURE;
            Rayon_De_Courbure(liste.rayon, liste.theta_ray, VMAX, AMAX, liste.sens, DMAX);
            if (finRayonCourbure){
                liste.type = (TYPE_MOUVEMENT_SUIVANT);
                finRayonCourbure = 0;
            }
/*            else if (stop_receive){
                liste.type = TYPE_INIT_STOP;
                //write_PWMD(0);
                //write_PWMG(0);
            }
*/            break;
              }
            case (TYPE_TRAIT_ENCH_RCV):{
                
                while(liste.enchainement !=2);
                
                #if (F_DBUG_TRAIT_ETAT)
                //Serial.println("ID_TRAIT");
                remplirStruct(ID_TRAIT, 1, 0x01, 0,0,0,0,0,0,0, DATArobot);
                writeStructInCAN(DATArobot);                             //CAN
                //CANenvoiMsg1Byte(ID_TRAIT, 1);
                #endif
                
                int compteurMvnt = 0;
                for(compteurMvnt = 0; compteurMvnt<nb_ordres; compteurMvnt++)
                {
                    #if (F_DBUG_TRAIT_ETAT)
                    //Serial.println("ID_TRAIT");
                    remplirStruct(ID_TRAIT, 2, 0x03, compteurMvnt,0,0,0,0,0,0, DATArobot);
                    writeStructInCAN(DATArobot);                             //CAN
                    ////CANenvoiMsg2x1Byte(ID_TRAIT, 3, compteurMvnt);
                    #endif
                    
                    switch(liste.type)
                    {
                        case (TYPE_TRAIT_ENCH_RCV):
                        case (TYPE_DEPLACEMENT_LIGNE_DROITE_EN):
                            trait_Mouvement_Elementaire_Gene(&liste);
                            liste.vinit = liste.vfin;
                            break;
                        case (TYPE_DEPLACEMENT_RAYON_COURBURE_CLOTHOIDE):
                            trait_Rayon_De_Courbure_Clotho(&liste);
                            liste.vinit = liste.vinit;
                            break;
                        
                    }
                    
                }
                
                
                liste.type = (TYPE_DEPLACEMENT_LIGNE_DROITE_EN);
                etat_automate_depl = (INITIALISATION);
                
                #if (F_DBUG_TRAIT_ETAT)
                //Serial.println("ID_TRAIT");
                remplirStruct(ID_TRAIT, 1, 0x02, 0,0,0,0,0,0,0, DATArobot);
                writeStructInCAN(DATArobot);                             //CAN
                //CANenvoiMsg1Byte(ID_TRAIT, 2);
                #endif
                    
            break;
            }            
            case (TYPE_DEPLACEMENT_LIGNE_DROITE_EN):{
            Message_Fin_Mouvement = (ASSERVISSEMENT_RECALAGE);
            Mouvement_Elementaire_Gene(liste);
            if (finMvtElem){
                liste.type = (TYPE_MOUVEMENT_SUIVANT);
                finMvtElem = 0;
            }
/*            else if (stop_receive){
                liste.type = TYPE_INIT_STOP;
                //write_PWMD(0);
                //write_PWMG(0);
            }
*/            break;
              }
            case (TYPE_DEPLACEMENT_RAYON_COURBURE_CLOTHOIDE) :{
            Message_Fin_Mouvement = (ASSERVISSEMENT_COURBURE);
            Rayon_De_Courbure_Clotho(liste);
            if (finRayonCourbureClo){
                liste.type = (TYPE_MOUVEMENT_SUIVANT);
                finRayonCourbureClo = 0;
            }
/*            else if (stop_receive){
                liste.type = TYPE_INIT_STOP;
                //write_PWMD(0);
                //write_PWMG(0);
            }
*/            break;
              }
            case (TYPE_DEPLACEMENT_RECALAGE) :{
            Message_Fin_Mouvement = (ASSERVISSEMENT_RECALAGE);
            Recalage(liste.distance, 3, 25, liste.recalage, liste.val_recalage); // ancienne Amax : 500
            if (finRecalage){
                liste.type = (TYPE_MOUVEMENT_SUIVANT);
                finRecalage = 0;
            }
            /*else if (stop_receive){
                liste.type = TYPE_INIT_STOP;
                write_PWMD(0);
                write_PWMG(0);
            }*/
            break;
            }
            case (TYPE_ASSERVISSEMENT_DESACTIVE) :{
            write_PWMD(0);
            write_PWMG(0); 
            if (asser_actif == 1){
                Asser_Init();
                liste.type = (TYPE_DEPLACEMENT_IMMOBILE);
            }
            break;
            }
            case (TYPE_DEPLACEMENT_BEZIER):{
             //Première valeur de la courbe
            if(flagDebutBezier == 0)
            {
                Message_Fin_Mouvement = (ASSERVISSEMENT_BEZIER);
                //Recuperation des valeurs dans le buffer
                buf_circ_pop(&buffer_distanceD, &distanceD);
                buf_circ_pop(&buffer_distanceG, &distanceG);
                flagDebutBezier = 1;
                nbValeurs = 0;
            }
            
            
            //Ne récupère pas de valeurs tant qu'on a pas utilisé autant de valeurs que FACTEUR_DIVISION
            if(nbValeurs >= (FACTEUR_DIVISION))
            {
                //Recuperation des valeurs dans le buffer
                buf_circ_pop(&buffer_distanceD, &distanceD);
                buf_circ_pop(&buffer_distanceG, &distanceG);
                nbValeurs = 0;
            }
            
            //Il y a de la place dans les buffer, demande nouvelles valeurs
            if(buf_circ_free_space(&buffer_distanceG) > 0)
            {
                //L'envoi d'un ack provoque l'envoi d'une nouvelle valeur
                //Serial.println("ACKNOWLEDGE_BEZIER");
                remplirStruct(ACKNOWLEDGE_BEZIER,0,0,0,0,0,0,0,0,0, DATArobot);
                writeStructInCAN(DATArobot);
                //CANenvoiMsg(ACKNOWLEDGE_BEZIER);
            }
            
            nbValeurs ++; //Permet de savoir le nombre de valeurs utilisées
            if(Courbe_bezier(distanceG/(FACTEUR_DIVISION), distanceD/(FACTEUR_DIVISION))) //Vrai si fin du mouvement
            {
                liste.type = (TYPE_MOUVEMENT_SUIVANT);
                flagDebutBezier = 0;
            }
/*            else if (stop_receive){
                liste.type = TYPE_INIT_STOP;
                //write_PWMD(0);
                //write_PWMG(0);
                flagDebutBezier = 0;
            }
*/            break;
              }
            
                
         /*   case TYPE_MOUVEMENT_SUIVANT :{
            //Remise a zero des variables
            Asser_Init();
            
            //Reinitialisation etat automate des mouvements
            etat_automate_depl = INITIALISATION;
            
            //On conserve la position du robot pour la prochaine action
            roue_drt_init = lireCodeurD();
            roue_gch_init = lireCodeurG();
            
            //CANenvoiMsg2x1Byte(INSTRUCTION_END_MOTEUR, Message_Fin_Mouvement, 0);
            
            //On vide le buffer de mouvements et on prévoit de s'asservir sur la position atteinte
            for(i = 0;i<nb_ordres;i++)  liste = (struct Ordre_deplacement){0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
            liste = (struct Ordre_deplacement){TYPE_DEPLACEMENT_IMMOBILE,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
            nb_ordres = 0;
            cpt_ordre = 0;
            cpt = 0;
            break;
            }*/
            
            case TYPE_INIT_STOP :{
            //Arret_Brutal();
            //cpt_stop = 0;
            //etat_automate_depl = DECELERATION_TRAPEZE;
            //cpt=global_ta_stop;
            Message_Fin_Mouvement = ASSERVISSEMENT_STOP;
            liste.type = TYPE_STOP;
            break;
            }
            case TYPE_STOP :{
/*            cpt_stop ++;
            if ((cpt_stop > 25)))
            {
                if(stop_receive==0)
                    liste.type = TYPE_MOUVEMENT_SUIVANT;
            }
*/            break;
            }
            default :{
              cmdD = Asser_Pos_MotD(roue_drt_init);
              cmdG = Asser_Pos_MotG(roue_gch_init);
              write_PWMG(cmdG);   
              write_PWMD(cmdD);
            break;
            }
        }
        
        if(liste.type == TYPE_MOUVEMENT_SUIVANT)
        {
        #if F_DBUG_ETAT && F_DBUG_ETAT_MS
            //CANenvoiMsg1Byte(ID_DBUG_ETAT, liste.type); a faire
        #endif
        
            
            //CANenvoiMsg4x1Byte(INSTRUCTION_END_MOTEUR, Message_Fin_Mouvement, 0, cpt_ordre, liste.enchainement); a faire
            //CANenvoiMsg3x2Bytes(ODOMETRIE_SMALL_POSITION, Odo_x, Odo_y, ((int16_t)Odo_theta) % 3600); a faire       
           
            if( liste.enchainement == 1)
            {
                //Remise a zero des variables
                //Asser_Init();
                //Reinitialisation etat automate des mouvements
                etat_automate_depl = INITIALISATION;
                
                //On conserve la position du robot pour la prochaine action
                
                consigne_pos = 0;
                roue_drt_init = lireCodeurD();
                roue_gch_init = lireCodeurG();
                
                cpt_ordre ++;
                cpt = 0;
                
            }
            else
            {
                //Remise a zero des variables
                Asser_Init();
                consigne_vit = 0;
                consigne_pos = 0;
                //Reinitialisation etat automate des mouvements
                etat_automate_depl = INITIALISATION;
                
                //On conserve la position du robot pour la prochaine action
                roue_drt_init = lireCodeurD();
                roue_gch_init = lireCodeurG();
                
                //On vide le buffer de mouvements et on prévoit de s'asservir sur la position atteinte
                liste = (struct Ordre_deplacement){TYPE_DEPLACEMENT_IMMOBILE,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                nb_ordres = 0;
                cpt_ordre = 0;
                cpt = 0;
            }
        }
    }
    else // on a recu un stop
    {
                Arret_Brutal();//Remise a zero des variables
                Asser_Init();
                consigne_vit = 0;
                consigne_pos = 0;
                //Reinitialisation etat automate des mouvements
                etat_automate_depl = INITIALISATION;
                etat_automate_xytheta = INIT_X_Y_THETA;
                //On conserve la position du robot pour la prochaine action
                roue_drt_init = lireCodeurD();
                roue_gch_init = lireCodeurG();
                //On vide le buffer de mouvements et on prévoit de s'asservir sur la position atteinte
                liste = (struct Ordre_deplacement){TYPE_DEPLACEMENT_IMMOBILE,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                nb_ordres = 0;
                cpt_ordre = 0;
                cpt = 0;
    }
    
    //On calcule l'odométrie à chaque tour de boucle
    //Odometrie();   
}

/***************************************************************************************
 NOM : CANloop                                                              
 ARGUMENT : aucun                                        
 RETOUR : rien                                                                        
 DESCRIPTIF :   Fonction principale appelé periodiquement, qui gère la communication avec le CAN et le Bluetooth
                Chaque id de message reçu, une tâche associé 
***************************************************************************************/
void CANloop(){
  if(canAvailable || BtAvailable){
    if(canAvailable){canReadExtRtr();}//On le me ici pour ne pas surcharger l'interruption CAN.onRecveive
    canAvailable = false; BtAvailable = false;
    //Serial.println("CAN received");
    
    switch (DATAtoControl.ID)
    {
            case ASSERVISSEMENT_REQUETE_PID:
                //Serial.println("ASSERVISSEMENT_REQUETE_PID");
                CANenvoiMsg1x8Bytes(ASSERVISSEMENT_CONFIG_KPP, &KppD);
                CANenvoiMsg1x8Bytes(ASSERVISSEMENT_CONFIG_KPI, &KipD);
                CANenvoiMsg1x8Bytes(ASSERVISSEMENT_CONFIG_KPD, &KdpD);
                break;
            case ASSERVISSEMENT_CONFIG_KPP_DROITE:
                memcpy(&KppD, DATAtoControl.dt, 8);
                KppDa = KppD;
                //Serial.println("ASSERVISSEMENT_CONFIG_KPP_DROITE");
                break;
            case ASSERVISSEMENT_CONFIG_KPI_DROITE:
                memcpy(&KipD, DATAtoControl.dt, 8);
                KipDa = KipD;
                //Serial.println("ASSERVISSEMENT_CONFIG_KPI_DROITE");
                break;
            case ASSERVISSEMENT_CONFIG_KPD_DROITE:
                memcpy(&KdpD, DATAtoControl.dt, 8);
                KdpDa = KdpD;
                //Serial.println("ASSERVISSEMENT_CONFIG_KPD_DROITE");
                break;
                
            case ASSERVISSEMENT_CONFIG_KPP_GAUCHE:
                memcpy(&KppG, DATAtoControl.dt, 8);
                KppGa = KppG;
                //Serial.println("ASSERVISSEMENT_CONFIG_KPP_GAUCHE");
                break;
            case ASSERVISSEMENT_CONFIG_KPI_GAUCHE :
                memcpy(&KipG, DATAtoControl.dt, 8);
                KipGa = KipG;
                //Serial.println("ASSERVISSEMENT_CONFIG_KPI_GAUCHE");
                break;
            case ASSERVISSEMENT_CONFIG_KPD_GAUCHE :
                memcpy(&KdpG, DATAtoControl.dt, 8);
                KdpGa = KdpG;
                //Serial.println("ASSERVISSEMENT_CONFIG_KPD_GAUCHE");
                break;
                
            case ASSERVISSEMENT_CONFIG_KPP:{
                Kp = 0;
                memcpy(&Kp, DATAtoControl.dt, 8);
                
                AsserInitCoefs(Kp, Ki, Kd);
                Serial.print("  ASSERVISSEMENT_CONFIG_KPP : ");
                Serial.printf("%f ", Kp);
                Serial.println();
                }
                break;
            case ASSERVISSEMENT_CONFIG_KPI :
                Ki = 0;
                memcpy(&Ki, DATAtoControl.dt, 8);
                AsserInitCoefs(Kp, Ki, Kd);
                Serial.print("  ASSERVISSEMENT_CONFIG_KPI : ");
                Serial.printf("%f ", Ki);
                Serial.println();
                break;
            case ASSERVISSEMENT_CONFIG_KPD :
                Kd = 0;
                memcpy(&Kd, DATAtoControl.dt, 8);
                AsserInitCoefs(Kp, Ki, Kd); 
                Serial.print("  ASSERVISSEMENT_CONFIG_KPD : ");
                Serial.printf("%f ", Kd);
                Serial.println();
                break;
//---------------------------------------------Qt
            case ASSERVISSEMENT_CONFIG_KPP_Qt:{
                Kp = ((DATAtoControl.dt[0] << 24) | (DATAtoControl.dt[1] << 16) | 
                      (DATAtoControl.dt[2] << 8) | DATAtoControl.dt[3]) / 1000.000;  
                
                AsserInitCoefs(Kp, Ki, Kd);
                Serial.print("  ASSERVISSEMENT_CONFIG_KPP Qt: ");
                Serial.printf("%f ", Kp);
                Serial.println();
                }
                break;
            case ASSERVISSEMENT_CONFIG_KPI_Qt :
                Ki = ((DATAtoControl.dt[0] << 24) | (DATAtoControl.dt[1] << 16) | 
                      (DATAtoControl.dt[2] << 8) | DATAtoControl.dt[3]) / 1000.000;  

                AsserInitCoefs(Kp, Ki, Kd);
                Serial.print("  ASSERVISSEMENT_CONFIG_KPI Qt: ");
                Serial.printf("%f ", Ki);
                Serial.println();
                break;
            case ASSERVISSEMENT_CONFIG_KPD_Qt :
                Kd= ((DATAtoControl.dt[0] << 24) | (DATAtoControl.dt[1] << 16) | 
                      (DATAtoControl.dt[2] << 8) | DATAtoControl.dt[3]) / 1000.000;  
                     
                AsserInitCoefs(Kp, Ki, Kd); 
                Serial.print("  ASSERVISSEMENT_CONFIG_KPD Qt: ");
                Serial.printf("%f ", Kd);
                Serial.println();
                break;
//---------------------------------------------
            case ASSERVISSEMENT_CONFIG_PERIMETRE_ROUE_CODEUSE :
                memcpy(&PERIMETRE_ROUE_CODEUSE, DATAtoControl.dt, 8);
                AsserInitCoefs(Kp, Ki, Kd); 
                Serial.print("  ASSERVISSEMENT_CONFIG_PERIMETRE_ROUE_CODEUSE : ");
                Serial.printf("%f ", PERIMETRE_ROUE_CODEUSE);
                Serial.println();
                break;
            case ASSERVISSEMENT_CONFIG_LARGEUR_ROBOT :
                memcpy(&LARGEUR_ROBOT, DATAtoControl.dt, 8);
                AsserInitCoefs(Kp, Ki, Kd); 
                Serial.print("  ASSERVISSEMENT_CONFIG_LARGEUR_ROBOT : ");
                Serial.printf("%f ", LARGEUR_ROBOT);
                Serial.println();
                break;
            case ECRAN_CHOICE_COLOR :
                Serial.println("ECRAN_CHOICE_COLOR");
                break;
                
            case ASSERVISSEMENT_ENABLE :
                asser_actif = DATAtoControl.dt[0];
                if(asser_actif == 1)
                {
                    roue_drt_init = lireCodeurD();
                    roue_gch_init = lireCodeurG();
                }
                Serial.println("ASSERVISSEMENT_ENABLE");
            break;
                
            case ASSERVISSEMENT_DECEL :
                {
                    double k = TE/0.02;
                    VMAX = ((double)DATAtoControl.dt[0]+256*DATAtoControl.dt[1])*k;
                    DMAX = ((double)DATAtoControl.dt[2]+256*DATAtoControl.dt[3])*k*k;
                    ralentare = 1;
                    //Serial.println("ACKNOWLEDGE_MOTEUR");
                    remplirStruct(ACKNOWLEDGE_MOTEUR, 2, 0x19, 0,0,0,0,0,0,0, DATArobot);
                    writeStructInCAN(DATArobot);
                    //CANenvoiMsg2x1Byte(ACKNOWLEDGE_MOTEUR, 0x19, 0);
                }
                //Serial.println("ASSERVISSEMENT_DECEL");
            break;
            case ASSERVISSEMENT_XYT :
            {
                    // `#START MESSAGE_X_Y_Theta_RECEIVED` 
                    stop_receive = 0;

                    //On vide le buffer de mouvements
                    liste = (struct Ordre_deplacement){TYPE_DEPLACEMENT_IMMOBILE,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                    nb_ordres = 0;
                    cpt_ordre = 0;//ne sert à rien mais au cas où, au futur...

                    liste.x = (DATAtoControl.dt[1] << 8) | DATAtoControl.dt[0];
                    liste.y = (DATAtoControl.dt[3] << 8) | DATAtoControl.dt[2];
                    liste.theta = (DATAtoControl.dt[5] << 8) | DATAtoControl.dt[4];
                    liste.sens =  DATAtoControl.dt[6];
                    liste.type = TYPE_DEPLACEMENT_X_Y_THETA;
                    liste.vmax = VMAX;
                    liste.amax = AMAX;        
                    //Serial.println("ACKNOWLEDGE_MOTEUR");
                    remplirStruct(ACKNOWLEDGE_MOTEUR, 2, 0x20, 0,0,0,0,0,0,0, DATArobot);
                    writeStructInCAN(DATArobot);    
                    //CANenvoiMsg2x1Byte(ACKNOWLEDGE_MOTEUR, 0x20, 0);
            }
            break;
            case ASSERVISSEMENT_COURBURE:
            {
                /* `#START MESSAGE_Rayon_de_courbure_RECEIVED` */
                stop_receive = 0;

                int16_t rayon = (DATAtoControl.dt[1] << 8) | DATAtoControl.dt[0];
                int16_t theta = (DATAtoControl.dt[3] << 8) | DATAtoControl.dt[2];
                uint8_t sens = DATAtoControl.dt[4];
                uint8_t enchainement = DATAtoControl.dt[5];
                uint8_t speedRatio = DATAtoControl.dt[6];
        
                if (enchainement)
                {
                    liste.type = TYPE_DEPLACEMENT_RAYON_COURBURE_CLOTHOIDE;
                    liste.rayon = rayon;
                    liste.theta_ray = theta;
                    liste.sens = sens;
                    liste.vmax = VMAX;
                    liste.amax = AMAX_CLO;
                    liste.dmax = DMAX;
                    liste.enchainement = enchainement;
                    liste.vinit = VMAX;//(long)((long)(VMAX)*(long)(speedRatio))>>8;
                    nb_ordres++;
            
                    if(enchainement == (2 || 1)) // ERREUR ?!?
                    {
                        //Serial.println("ACKNOWLEDGE_MOTEUR");
                        remplirStruct(ACKNOWLEDGE_MOTEUR, 2, 0x21, 0,0,0,0,0,0,0, DATArobot);
                        writeStructInCAN(DATArobot);
                        //CANenvoiMsg2x1Byte(ACKNOWLEDGE_MOTEUR, 0x21, 0 /* enchainement<<3 */);                
                    }
                }
                else
                {
                    liste = (struct Ordre_deplacement){TYPE_DEPLACEMENT_IMMOBILE,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                    nb_ordres = 0;
                    cpt_ordre = 0;
                    liste.type = TYPE_DEPLACEMENT_RAYON_COURBURE;
                    liste.rayon = rayon;
                    liste.theta_ray = theta;
                    liste.sens = sens;
                    liste.vmax = VMAX;
                    liste.amax = AMAX;
                    liste.enchainement = enchainement;
                    //Serial.println("ACKNOWLEDGE_MOTEUR");
                    remplirStruct(ACKNOWLEDGE_MOTEUR, 2, 0x21, 0,0,0,0,0,0,0, DATArobot);
                    writeStructInCAN(DATArobot);
                    //CANenvoiMsg2x1Byte(ACKNOWLEDGE_MOTEUR, 0x21, 0);
                }
        
            }
            break;  
            case ASSERVISSEMENT_CONFIG:
            {
                double k = TE/0.02;
                int16_t vmax = (DATAtoControl.dt[1] << 8) | DATAtoControl.dt[0];
                int16_t amax = (DATAtoControl.dt[3] << 8) | DATAtoControl.dt[2];
                VMAX = ((double)vmax)*k;
                AMAX = ((double)amax)*k*k;
                DMAX = ((double)amax)*0.75*k*k;
                ralentare = 1;
        
                //CANenvoiMsg2x1Byte(ACKNOWLEDGE_MOTEUR, 0x22, 0);
                remplirStruct(ACKNOWLEDGE_MOTEUR, 2, 0x22, 0,0,0,0,0,0,0, DATArobot);
                writeStructInCAN(DATArobot);
            }
            break;
            case ASSERVISSEMENT_ROTATION:
            {
                /* `#START MESSAGE_Rotation_RECEIVED` */
                stop_receive = 0;

                liste = (struct Ordre_deplacement){TYPE_DEPLACEMENT_IMMOBILE,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                nb_ordres = 0;
                cpt_ordre = 0;

                int16_t angle = (DATAtoControl.dt[1] << 8) | DATAtoControl.dt[0];
        
                liste.type = TYPE_DEPLACEMENT_ROTATION;
                liste.angle = LARGEUR_ROBOT * M_PI * RESOLUTION_ROUE_CODEUSE * angle / (3600 * PERIMETRE_ROUE_CODEUSE);
                liste.vmax = VMAX;
                liste.amax = AMAX;
                //Serial.println("ACKNOWLEDGE_MOTEUR");
                remplirStruct(ACKNOWLEDGE_MOTEUR, 2, 0x23, 0,0,0,0,0,0,0, DATArobot);
                writeStructInCAN(DATArobot);
                //CANenvoiMsg2x1Byte(ACKNOWLEDGE_MOTEUR, 0x23, 0);
                
            }
            break;  
            case ASSERVISSEMENT_RECALAGE:
            {
                stop_receive = 0;

                int16_t distance = (DATAtoControl.dt[1] << 8) | DATAtoControl.dt[0];
                uint8_t mode = DATAtoControl.dt[2];
                int16_t valRecalage = (DATAtoControl.dt[4] << 8) | DATAtoControl.dt[3];
                uint8_t enchainement = DATAtoControl.dt[5];
                int8_t vinit = DATAtoControl.dt[6];
                int8_t vfin = DATAtoControl.dt[7];
        
                //Recalage
                if (mode)
                {
                    liste = (struct Ordre_deplacement){TYPE_DEPLACEMENT_IMMOBILE,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                    nb_ordres = 0;
                    cpt_ordre = 0;
                    liste.type = TYPE_DEPLACEMENT_RECALAGE;
                    liste.distance = ((distance * RESOLUTION_ROUE_CODEUSE) / PERIMETRE_ROUE_CODEUSE); //-
                    liste.vmax = 10;
                    liste.amax = 100;
                    liste.enchainement = 0;
                    liste.val_recalage = valRecalage;
                    liste.recalage = mode;

                    remplirStruct(ACKNOWLEDGE_MOTEUR, 2, 0x24, 0,0,0,0,0,0,0, DATArobot);
                    writeStructInCAN(DATArobot);
                    //CANenvoiMsg2x1Byte(ACKNOWLEDGE_MOTEUR, 0x24, 0);
                }
        
                //Enchainement
                else if (enchainement)
                {
                    if(nb_ordres) liste.type = TYPE_DEPLACEMENT_LIGNE_DROITE_EN;
                    else liste.type = TYPE_TRAIT_ENCH_RCV;
                    liste.distance = (distance * RESOLUTION_ROUE_CODEUSE) / PERIMETRE_ROUE_CODEUSE;
                    liste.vmax = VMAX;
                    liste.amax = AMAX;
                    liste.dmax = DMAX;
                    liste.enchainement = enchainement;
                    liste.vinit = (long)((long)(VMAX)*(long)(vinit))>>8;
                    liste.vfin = (long)((long)(VMAX)*(long)(vfin))>>8;
            
                    if (nb_ordres ==0 ){
                        liste.vinit = 0;
                        liste.vfin = VMAX;
                    } else if (enchainement == 2){
                        liste.vinit = VMAX;
                        liste.vfin = 0;
                    } else {
                        liste.vinit = VMAX;
                        liste.vfin = VMAX;
                    }
                    nb_ordres++;
            
                    if (enchainement == 2 ||1)  // ERREUR ?!?
                    {
                        remplirStruct(ACKNOWLEDGE_MOTEUR, 2, 0x24, 0,0,0,0,0,0,0, DATArobot);
                        writeStructInCAN(DATArobot);
                        //CANenvoiMsg2x1Byte(ACKNOWLEDGE_MOTEUR, 0x24, 0 /* enchainement<<3 */);
                    }
                }
        
                //Ligne droite
                else
                {          
                    //On vide le buffer de mouvements
                    liste = (struct Ordre_deplacement){TYPE_DEPLACEMENT_IMMOBILE,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                    nb_ordres = 0;
                    cpt_ordre = 0;
                    liste.type = TYPE_DEPLACEMENT_LIGNE_DROITE;
                    liste.distance = (distance * RESOLUTION_ROUE_CODEUSE) / PERIMETRE_ROUE_CODEUSE;
                    liste.vmax = VMAX;
                    liste.amax = AMAX;
                    liste.enchainement = enchainement;

                    remplirStruct(ACKNOWLEDGE_MOTEUR, 2, 0x24, 0,0,0,0,0,0,0, DATArobot);
                    writeStructInCAN(DATArobot);
                    //CANenvoiMsg2x1Byte(ACKNOWLEDGE_MOTEUR, 0x24, 0);
                }
            }
            break;
            case ASSERVISSEMENT_BEZIER:
            {
                 /* `#START MESSAGE_Courbe_Bezier_RECEIVED` */
        
                // distance roue droite en ticks d'encodeur
                int32_t distRoueDroite = (DATAtoControl.dt[3] << 24) | (DATAtoControl.dt[2] << 16) | (DATAtoControl.dt[1] << 8) | DATAtoControl.dt[0];          //distance roue droite(4/4) |
                // distance roue gauche en ticks d'encodeur
                int32_t distRoueGauche = (DATAtoControl.dt[7] << 24) | (DATAtoControl.dt[6] << 16) | (DATAtoControl.dt[5] << 8) | DATAtoControl.dt[4];
        
                //Reception premiere valeur de la courbe
                if(flagDebutBezier == 0)
                {
                    liste.type = TYPE_DEPLACEMENT_BEZIER;
                }

                //Stockage des valeurs dans les buffer
                buf_circ_push(&buffer_distanceD, distRoueDroite);
                buf_circ_push(&buffer_distanceG, distRoueGauche);

                //Il y a de la place dans les buffer, demande nouvelles valeurs
                if(buf_circ_free_space(&buffer_distanceG) > 0)
                {
                    //L'envoi d'un ack provoque l'envoi d'une nouvelle valeur
                    //Serial.println("ACKNOWLEDGE_BEZIER");
                    remplirStruct(ACKNOWLEDGE_BEZIER,0,0,0,0,0,0,0,0,0, DATArobot);
                    writeStructInCAN(DATArobot);

                    //CANenvoiMsg(ACKNOWLEDGE_BEZIER);
                }
        
        

            }
            break;  
            case ODOMETRIE_SMALL_POSITION:
            {
                Odo_x = (DATAtoControl.dt[1] << 8) | DATAtoControl.dt[0];
                Odo_y = (DATAtoControl.dt[3] << 8) | DATAtoControl.dt[2];
                Odo_theta = (DATAtoControl.dt[5] << 8) | DATAtoControl.dt[4];
            }
            break;
            case ODOMETRIE_SMALL_VITESSE:
            {

            }
            break;
            case ODOMETRIE_BIG_POSITION:
            {
                Odo_x = (DATAtoControl.dt[1] << 8) | DATAtoControl.dt[0];
                Odo_y = (DATAtoControl.dt[3] << 8) | DATAtoControl.dt[2];
                Odo_theta = (DATAtoControl.dt[5] << 8) | DATAtoControl.dt[4];
            }
            break;
            case ODOMETRIE_BIG_VITESSE:
            {

            }
            break;
            case GLOBAL_GAME_END:
            {
                 /* `#START MESSAGE_End_Game_RECEIVED` */
                Fin_Match = 1;
            }
            break;
            case ASSERVISSEMENT_STOP:
            {
                /* `#START MESSAGE_Stop_RECEIVED` */
                stop_receive = 1;
                //Serial.println("ACKNOWLEDGE_MOTEUR");
                remplirStruct(ACKNOWLEDGE_MOTEUR, 2, 0x01, 0,0,0,0,0,0,0, DATArobot);
                writeStructInCAN(DATArobot);
                //CANenvoiMsg2x1Byte(ACKNOWLEDGE_MOTEUR, 0x01, 0);
            }
            break;  
            case CHECK_MOTEUR:
            {
                /* `#START MESSAGE_Check_RECEIVED` */
                attente = 1;
                //Serial.println("ALIVE_MOTEUR");
                remplirStruct(ALIVE_MOTEUR, 0, 0, 0,0,0,0,0,0,0, DATArobot);
                writeStructInCAN(DATArobot);
                //CANenvoiMsg(ALIVE_MOTEUR);
            }
            break;

            case 0x00:
            {
                
            }
            break;  
            
            default :
            
            break;
    }
    // Send message to master via bleutooth
    /*if (connected){
      prxRemoteCharacteristic->writeValue((uint8_t *)&DATAtoControl, sizeof(DATAtoControl));
      Serial.println("Sending via BT...");
    } else{Serial.println("The device is not connected");}*/
  }
  //if new CAN by BT are available, write it in CAN bus / CAN <-> Bt <-> CAN and use it to control the Robot
  if (newCan){
    newCan = false;
    writeStructInCAN(DATAtoControl); 
    Serial.println(DATAtoControl.ID);
    BtAvailable = true;
  }
  
}

/***************************************************************************************
 NOM : Mouvement Elementaire                                                           
 ARGUMENT : long pcons -> distance a parcourir (>0 : avancer et <0 : reculer          
            short vmax -> vitesse maximale                                            
            short amax -> acceleration maximale                                       
            short dmax -> deceleration maximale                                      
            char mvt -> choix entre rotation et ligne droite                          
 RETOUR : rien                                                                        
 DESCRIPTIF : Fonction appelee pour effectuer une ligne droite                        
***************************************************************************************/
void Mouvement_Elementaire(long pcons, short vmax, short amax, short dmax, char mvt)
{    
    //Declaration des variables
    static double tc, ta, td;   //tc temps à vitesse constante, ta en acceleration, td en deceleration
    static double vmax_tri=1;     //Vitesse maximale atteinte dans le cas d'un profil en triangle
    static double accel = 1, decel = 1;
    double pos_triangle;    //Position triangle avec vitesse max
    static short memo_etat_automate = 0;
    static short defaut_asserv = 0;
    
    switch(etat_automate_depl)      //Automate de gestion du deplacement
    {
     case INITIALISATION :      //Etat d'initialisation et de calcul des variables
        //Remise a zero des variables car on effectue un nouveau deplacement
        consigne_pos = 0;
        consigne_vit = 0;
        cpt = 0;
        defaut_asserv = 0;
        
        accel = amax * 0.001;
        decel = dmax * 0.001;
        //Elaboration du profil de vitesse
        ta = vmax / accel;      //Calcul du temps d'acceleration
        td = vmax / decel;      //Calcul du temps de deceleration
        global_ta_stop=ta;
        global_decel_stop=5*decel;
        pos_triangle = (0.5 * vmax) * (ta + td);
        
        long posCalc;
        if (pos_triangle < fabs((double)pcons)){
            //Profil trapeze
            tc = (fabs((double)pcons) - pos_triangle) / (double)vmax;
            etat_automate_depl = ACCELERATION_TRAPEZE;
            posCalc = ta*vmax/2+ td*vmax/2 + tc*vmax;
        }
        
        else{
            //Profil triangle
            vmax_tri = sqrt(2.0 * fabs((double)pcons) / (1.0/accel+1.0/decel));
            ta = vmax_tri / accel;      //Calcul du temps d'acceleration
            td = vmax_tri / decel;      //Calcul du temps de deceleration
            global_ta_stop = ta;
            global_decel_stop=5*decel;
            etat_automate_depl = ACCELERATION_TRIANGLE;
            posCalc = ta*vmax_tri/2 + td*vmax_tri/2;
        }
        //Serial.println("ID_DIST_TIC_GENE");
        remplirStruct(ID_DIST_TIC_GENE, 2, (pcons&0xFF), ((pcons&0xFF00)<<8),0,0,0,0,0,0, DATArobot);
        writeStructInCAN(DATArobot);
        
        #if F_DBUG_LIGNE
        //CANenvoiMsg4x2Bytes(ID_DBUG_LIGNE_TPS, etat_automate_depl, ta, td, tc);
                
        //CANenvoiMsg4Bytes(ID_DBUG_LIGNE_PCONS, &pcons);
                        
        //CANenvoiMsg3x2Bytes(ID_DBUG_LIGNE_VIT, vmax, amax, dmax);
        #endif
        
        break;
        
     case ACCELERATION_TRAPEZE :    //Etat d'acceleration en profil trapeze
        cpt ++;
        
        //Incrementation de la consigne de vitesse par la valeur de l'acceleration
        consigne_vit += accel;          
        
        //Incrementation de la consigne de position
        if (pcons>0) {
            consigne_pos += consigne_vit;
        } else {
            consigne_pos -= consigne_vit;
        }
        
        if(cpt >= ta)   //Condition pour quitter la phase d'acceleration
        {
            etat_automate_depl = VITESSE_CONSTANTE_TRAPEZE;      //Passage a l'etat VITESSE_CONSTANTE
            cpt = 0;
            consigne_vit = vmax;
        }
/*        if(stop_receive)
            etat_automate_depl=ARRET_STOP;
*/        break;
        
     case VITESSE_CONSTANTE_TRAPEZE :   //Etat de vitesse constante en profil trapeze
        cpt ++; 
        //Incrementation de la consigne de position
        if (pcons>0) {
            consigne_pos += consigne_vit;
        } else {
            consigne_pos -= consigne_vit;
        }
        
        //Si il n'y a pas d'enchainements
        if(cpt >= tc)     //Condition pour quitter la phase de vitesse constante
        {
            etat_automate_depl = DECELERATION_TRAPEZE;      //Passage a l'etat DECELERATION
            cpt = 0;
        }
/*        if(stop_receive)
            etat_automate_depl=ARRET_STOP;
*/
        break;
        
     case DECELERATION_TRAPEZE :    //Etat de deceleration en profil trapeze
        cpt ++;
         
        //Incrementation de la consigne de vitesse par la valeur de l'acceleration
        consigne_vit -= decel;          
        
        //Incrementation de la consigne de position
        if (pcons>0) 
        {
            consigne_pos += consigne_vit;
        } 
        else 
        {
            consigne_pos -= consigne_vit;
        }
        
        if(cpt >= td)       //Condition pour quitter la phase de deceleration en profil trapeze
        {
            etat_automate_depl = ARRET;     //Passage a l'etat ARRET
            consigne_pos = pcons;
            consigne_vit = 0;
            cpt = 0;
        }
/*        if(stop_receive)
            etat_automate_depl=ARRET_STOP;
*/        break;
        
     case ARRET :       //Etat d'arret
        cpt ++;
        
        if(cpt >= 20)       //Condition pour sortir de l'etat arret
        {
            finMvtElem = 1;
            etat_automate_depl = INITIALISATION;        //Passage a l'etat INITIALISATION
        }
        break;
        
    case ACCELERATION_TRIANGLE :        //Etat d'acceleration en profil triangle
        cpt ++;
        
        //Incrementation de la consigne de vitesse par la valeur de l'acceleration
        consigne_vit += accel;          
        
        //Incrementation de la consigne de position
        if (pcons>0) {
            consigne_pos += consigne_vit;
        } else {
            consigne_pos -= consigne_vit;
        }
        
        if(cpt >= ta)   //Condition pour quitter la phase d'acceleration
        {
            etat_automate_depl = DECELERATION_TRIANGLE;      //Passage a l'etat VITESSE_CONSTANTE
            cpt = 0;
            consigne_vit = vmax_tri;
        }
/*        if(stop_receive)
            etat_automate_depl=ARRET_STOP;
*/        break;
        
    case DECELERATION_TRIANGLE :        //Etat de deceleration en profil triangle
        cpt ++;
         
        //Incrementation de la consigne de vitesse par la valeur de l'acceleration
        consigne_vit -= decel;          
        
        //Incrementation de la consigne de position
        if (pcons>0) {
            consigne_pos += consigne_vit;
        } else {
            consigne_pos -= consigne_vit;
        }
        
        if(cpt >= td)       //Condition pour quitter la phase de deceleration en profil trapeze
        {
            etat_automate_depl = ARRET;     //Passage a l'etat ARRET
            consigne_pos = pcons;
            consigne_vit = 0;
            cpt = 0;
        }
/*        if(stop_receive)
            etat_automate_depl=ARRET_STOP;
*/        break;
        
    /*case TROP_D_ERREUR_ASSERV :
        etat_automate_depl = memo_etat_automate;
        defaut_asserv++;
        break;*/
/*    case ARRET_STOP:
        while(1)
            Arret_Brutal();
        if((old_posD==lireCodeurD())&&(old_posG==lireCodeurG()))
        {
            cpt_arret++;
            if(cpt_arret>100)
                etat_automate_depl = INITIALISATION;
                
        }
        else
        {
            cpt_arret=0;
            old_posD=lireCodeurD();
            old_posG=lireCodeurG();
        }
        break;
*/        
    default:
    break;
    }
    
    if((etat_automate_depl != INITIALISATION)&&(etat_automate_depl != ARRET_STOP))
    {
        //Calcul des commandes
        double cmdD, cmdG, erreur;
        
        if (mvt == MOUVEMENT_LIGNE_DROITE){
            /*cmdD = Asser_Pos_MotD(roue_drt_init + consigne_pos);
            cmdG = Asser_Pos_MotG(roue_gch_init + consigne_pos);*/
            Asser_Pos_Mot(roue_gch_init + consigne_pos, roue_drt_init + consigne_pos, &cmdG, &cmdD);
            erreur = ErreurPosG;
        }else{
            cmdD = Asser_Pos_MotD(roue_drt_init + consigne_pos);
            cmdG = Asser_Pos_MotG(roue_gch_init - consigne_pos);
            erreur = -ErreurPosG;
        }
        
        /*if (fabs(ErreurPosD + erreur) > EXPLOSION_TAUX) {
            // Trop d'écart
            memo_etat_automate = etat_automate_depl;
            if (defaut_asserv<3) {
                etat_automate_depl = TROP_D_ERREUR_ASSERV;
                //defaut_asserv++;
            } else {
                defaut_asserv = 0;
                asser_actif = 0;
                liste[cpt_ordre].type = TYPE_ASSERVISSEMENT_DESACTIVE;//msg defaut???
            }
        }*/
        //Ecriture du PWM sur chaque modeur
        write_PWMG(cmdG);   
        write_PWMD(cmdD);
        
        //Arret si le robot est bloqué
        lectureErreur();
    }
}
/***************************************************************************************
 NOM : Rayon_De_Courbure                                                              
 ARGUMENT : short rayon -> rayon de l'arc de cercle a parcourir                       
            short theta -> angle à parcourir                      // dixième de degré                    
            short vmax -> vitesse maximale                        // tic/TE                  
            short dmax -> deceleration maximale                   // tic/(TE)²               
            short amax -> acceleration maximale                   // tic/(TE)²                    
            short sens -> sens de deplacement                     // sens = 1 tourner a gauche   -1 trouner à droite                    
 RETOUR : rien                                                                        
 DESCRIPTIF : Fonction appelee pour effectuer un arc de cercle                        
              Dans un rayon de courbure, on avance en suivant un arc de cercle        
              situé à une distance rayon du centre du robot.                          
              On va donc faire comme avec une ligne droite, sauf qu'une des roues     
              (la roue intérieure dans le cercle) va moins avancer que l'autre        
              Le rapport de distance entre les 2 roues est :                          
              distance de la roue interieure par rapport au centre du cercle,         
              divisé par distance de la roue exterieur par rapport au centre,         
              Ou encore (2*(rayon - (largeur du robot/2))) / (2*(rayon + (largeur du robot/2)))  
              Ou encore (2*rayon - largeur) / (2*rayon + largeur)                     
              Pour le reste, c'est comme une ligne droite,                            
              Sauf que la distance à parcourir (pour la ext) est                      
              celle de l'arc de cercle : (rayon + largeur/2) * angle à parcourir      
***************************************************************************************/
void Rayon_De_Courbure(short rayon, short theta, short vmax, short amax, short sens, short dmax) //fait
{
    //Declaration des variables
    static double tc, ta, td;   //tc temps à vitesse constante, ta en acceleration, td en deceleration
    static double vmax_tri;     //Vitesse maximale atteinte dans le cas d'un profil en triangle
    static double accel = 0, decel = 0;
    static double pcons = 0, rapport = 0;
    double pos_triangle;    //Position triangle avec vitesse max
    static short memo_etat_automate = 0;
    static short defaut_asserv = 0;
    
    
    switch(etat_automate_depl)      //Automate de gestion du deplacement
    {
     case INITIALISATION :      //Etat d'initialisation et de calcul des variables
        //Remise a zero des variables car on effectue un nouveau deplacement
        consigne_pos = 0;
        consigne_vit = 0;
        cpt = 0;
        
        //Calcul de la position voulue
        pcons = ((rayon + (LARGEUR_ROBOT-0.5)/2.0) * 2.0 * M_PI * theta / 3600.0) * RESOLUTION_ROUE_CODEUSE / PERIMETRE_ROUE_CODEUSE;
                
        //Rapport pour la roue qui parcourt la plus petite distance   
        rapport = (rayon - (LARGEUR_ROBOT+0.5)/2.0)/(rayon + (LARGEUR_ROBOT-0.5)/2.0);
        
        accel = amax * 0.001;
        decel = dmax * 0.001;
        
        //Elaboration du profil de vitesse
        ta = vmax / accel;      //Calcul du temps d'acceleration
        td = vmax / decel;      //Calcul du temps de deceleration
        
        pos_triangle = (0.5 * vmax) * (ta + td);
        
        if (pos_triangle < fabs((double)pcons)){
            //Profil trapeze
            tc = (fabs((double)pcons) - pos_triangle) / (double)vmax;
            etat_automate_depl = ACCELERATION_TRAPEZE;
        }
        
        else{
            //Profil triangle
            vmax_tri = sqrt(2.0 * fabs((double)pcons) / (1.0/accel+1.0/decel));
            ta = vmax_tri / accel;      //Calcul du temps d'acceleration
            td = vmax_tri / decel;      //Calcul du temps de deceleration
            etat_automate_depl = ACCELERATION_TRIANGLE;
        }
        
        break;
        
     case ACCELERATION_TRAPEZE :    //Etat d'acceleration en profil trapeze
        cpt ++;
        
        //Incrementation de la consigne de vitesse par la valeur de l'acceleration
        consigne_vit += accel;          
        
        //Incrementation de la consigne de position
        if (pcons>0) {
            consigne_pos += consigne_vit;
        } else {
            consigne_pos -= consigne_vit;
        }
        
        if(cpt >= ta)   //Condition pour quitter la phase d'acceleration
        {
            etat_automate_depl = VITESSE_CONSTANTE_TRAPEZE;      //Passage a l'etat VITESSE_CONSTANTE
            cpt = 0;
            consigne_vit = vmax;
        }
        break;
        
     case VITESSE_CONSTANTE_TRAPEZE :   //Etat de vitesse constante en profil trapeze
        cpt ++; 
        //Incrementation de la consigne de position
        if (pcons>0) {
            consigne_pos += consigne_vit;
        } else {
            consigne_pos -= consigne_vit;
        }
        
        //Si il n'y a pas d'enchainements
        if(cpt >= tc)     //Condition pour quitter la phase de vitesse constante
        {
            etat_automate_depl = DECELERATION_TRAPEZE;      //Passage a l'etat DECELERATION
            cpt = 0;
        }
        break;
        
     case DECELERATION_TRAPEZE :    //Etat de deceleration en profil trapeze
        cpt ++;
         
        //Incrementation de la consigne de vitesse par la valeur de l'acceleration
        consigne_vit -= decel;          
        
        //Incrementation de la consigne de position
        if (pcons>0) {
            consigne_pos += consigne_vit;
        } else {
            consigne_pos -= consigne_vit;
        }
        
        if(cpt >= td)       //Condition pour quitter la phase de deceleration en profil trapeze
        {
            etat_automate_depl = ARRET;     //Passage a l'etat ARRET
            consigne_pos = pcons;
            consigne_vit = 0;
            cpt = 0;
        }
        break;
        
     case ARRET :       //Etat d'arret
        cpt ++;
        
        if(cpt >= 20)       //Condition pour sortir de l'etat arret
        {
            finRayonCourbure = 1;
            etat_automate_depl = INITIALISATION;        //Passage a l'etat INITIALISATION
        }
        break;
        
    case ACCELERATION_TRIANGLE :        //Etat d'acceleration en profil triangle
        cpt ++;
        
        //Incrementation de la consigne de vitesse par la valeur de l'acceleration
        consigne_vit += accel;          
        
        //Incrementation de la consigne de position
        if (pcons>0) {
            consigne_pos += consigne_vit;
        } else {
            consigne_pos -= consigne_vit;
        }
        
        if(cpt >= ta)   //Condition pour quitter la phase d'acceleration
        {
            etat_automate_depl = DECELERATION_TRIANGLE;      //Passage a l'etat VITESSE_CONSTANTE
            cpt = 0;
            consigne_vit = vmax_tri;
        }
        break;
        
    case DECELERATION_TRIANGLE :        //Etat de deceleration en profil triangle
        cpt ++;
         
        //Incrementation de la consigne de vitesse par la valeur de l'acceleration
        consigne_vit -= decel;          
        
        //Incrementation de la consigne de position
        if (pcons>0) {
            consigne_pos += consigne_vit;
        } else {
            consigne_pos -= consigne_vit;
        }
        
        if(cpt >= td)       //Condition pour quitter la phase de deceleration en profil trapeze
        {
            etat_automate_depl = ARRET;     //Passage a l'etat ARRET
            consigne_pos = pcons;
            consigne_vit = 0;
            cpt = 0;
        }
        break;
    
    case TROP_D_ERREUR_ASSERV :
        etat_automate_depl = memo_etat_automate;
        defaut_asserv++;
        break;
        
    
    default:
    break;
    }
    
    if (etat_automate_depl != INITIALISATION){
        double cmdD, cmdG;
        //Determination des commandes en fonction de la direction de deplacement du robot
        if(sens >= 0)  
        {
            Asser_Pos_Mot(roue_gch_init + (consigne_pos * rapport), roue_drt_init + consigne_pos, &cmdG, &cmdD); 
            //cmdD = Asser_Pos_MotD(roue_drt_init + consigne_pos);
            //cmdG = Asser_Pos_MotG(roue_gch_init + (consigne_pos * rapport));
        }else{
            Asser_Pos_Mot(roue_gch_init + consigne_pos, roue_drt_init + (consigne_pos * rapport), &cmdG, &cmdD); 
            //cmdD = Asser_Pos_MotD(roue_drt_init + (consigne_pos * rapport));
            //cmdG = Asser_Pos_MotG(roue_gch_init + consigne_pos);
        }
        
/*        if (fabs(ErreurPosD - ErreurPosG) > EXPLOSION_TAUX) 
        {
            // Trop d'écart
            memo_etat_automate = etat_automate_depl;
            if (defaut_asserv<3) {
                etat_automate_depl = TROP_D_ERREUR_ASSERV;
            } else {
                defaut_asserv = 0;//msg defaut???
            }
        }*/
        
        //Envoi de la vitesse aux moteurs
        write_PWMG(cmdG);   
        write_PWMD(cmdD);
        //Arret si le robot est bloqué
        lectureErreur();
    }
}
void trait_Mouvement_Elementaire_Gene(struct Ordre_deplacement* monDpl)//fait
{
    /*
    long pcons = monDpl.distance;
    short vinit = monDpl.vinit;
    short vfin = monDpl.vfin;*/
    
    
    
    
    
    long pconsAbs = fabs((double)monDpl->distance);
    short vmax = monDpl->vmax;
    short amax = monDpl->amax;
    short dmax = monDpl->dmax;
    
    double tc, ta, td;   //tc temps à vitesse constante, ta en acceleration, td en deceleration
    double vmax_tri, vmax_trap;     //Vitesse maximale atteinte dans le cas d'un profil en triangle
    double accel, decel;
    double pos_triangle, pos_vfin, pos_vmax;    //Position triangle avec vitesse max
    short vinitAbs, vfinAbs, vinitbAbs;
    long posCalc;
    
    if(signesDif(monDpl->distance, monDpl->vinit)){ monDpl->vinit *= -1;}
    if(signesDif(monDpl->distance, monDpl->vfin)) {monDpl->vfin *= -1;}
    
    accel = monDpl->amax * 0.001;
    decel = monDpl->dmax * 0.001;
    vmax_trap=monDpl->vmax;
    vmax_tri=monDpl->vmax;
    
    #if F_DBUG_TRAIT_ETAT_GENE
    //CANenvoiMsg1Byte(ID_TRAIT_LIGNE_GENE, 1);
    #endif
    
    vinitAbs = fabs((double)monDpl->vinit);
    vfinAbs = fabs((double)monDpl->vfin);
    
    if(vinitAbs < vfinAbs) pos_vfin = 0.5*(vfinAbs*vfinAbs-vinitAbs*vinitAbs) / accel;
    else pos_vfin = 0.5*(vinitAbs*vinitAbs-vfinAbs*vfinAbs) / decel;
    
    if(pos_vfin > pconsAbs) // Verification pour eviter un changement de vitesse trop important
    {
        if(vinitAbs == 0)
        {
            vfinAbs = sqrt(2*accel*pconsAbs);
            if(monDpl->distance>=0)monDpl->vfin = vfinAbs;
            else monDpl->vfin = -vfinAbs;
        }
        else if(vfinAbs == 0)
        {
            //WARNING!!!!!!!!!!!!!
            Serial.println("vfinAbs =0! Warning!");
        }
        else if(vfinAbs > vinitAbs)
        {
            vfinAbs = sqrt(vinitAbs*vinitAbs + 2*accel*pconsAbs);
            if(monDpl->distance>=0)monDpl->vfin = vfinAbs;
            else monDpl->vfin = -vfinAbs;
        }
        else if (vfinAbs <= vinitAbs)
        {
            vfinAbs = sqrt(vinitAbs*vinitAbs - 2*accel*pconsAbs);
            if(monDpl->distance>=0)monDpl->vfin = vfinAbs;
            else monDpl->vfin = -vfinAbs;
        }
    }
    
    //Elaboration du profil de vitesse
    ta = (vmax-vinitAbs) / accel;      //Calcul du temps d'acceleration
    td = (vmax-vfinAbs) / decel;      //Calcul du temps de deceleration
    
    pos_triangle = ta*0.5*(vmax+vinitAbs) + td*0.5*(vmax+vfinAbs);
    
    if (pos_triangle < pconsAbs){
        tc = (fabs((double)monDpl->distance) - pos_triangle) / (double)vmax;
        monDpl->vmax = vmax;
        if(monDpl->distance<0)
            vmax_trap *= -1;
        etat_automate_depl = ACCELERATION_TRAPEZE;
        posCalc = ta*(vmax+vinitAbs)/2+ td*(vmax+vfinAbs)/2 + tc*vmax;
    }
    else
    {
        vmax_tri =sqrt((accel*decel*2*fabs((double)monDpl->distance)+ accel*vfinAbs*vfinAbs+ decel*vinitAbs*vinitAbs)/(accel+decel));
        ta = (vmax_tri-vinitAbs) / accel; //Calcul du temps d'acceleration
        td = (vmax_tri-vfinAbs) / decel;  //Calcul du temps de deceleration
        tc = 0;
        etat_automate_depl = ACCELERATION_TRIANGLE;
        posCalc = ta*(vmax_tri+vinitAbs)/2+ td*(vmax_tri+vfinAbs)/2;
        if(monDpl->distance<0)monDpl->vmax = -vmax_tri;
        else monDpl->vmax = vmax_tri;
    }
    
    #if F_DBUG_LIGNE_GEN_TPS
    // CANenvoiMsg4x2Bytes(ID_TEMPS, ta, tc, td, 0x100 | (0xFF & etat_automate_depl));
    
    CANenvoiMsg2x4Bytes(ID_TEMPS_LONG_1, &ta, &tc);
    
    //CANenvoiMsg1x4Bytes(ID_TEMPS_LONG_2, &td);
    
    CANenvoiMsg2x4Bytes(ID_DBUG_LIGNE_GENE_VIT, &vinitAbs, &vfinAbs); 
    
    CANenvoiMsg2x4Bytes(ID_DIST_TIC_GENE, &monDpl->distance, &posCalc);
    
    #endif
    
    
    monDpl->ta =ta;
    monDpl->tc =tc;
    monDpl->td =td;
    
    
    #if F_DBUG_TRAIT_ETAT_GENE
    //CANenvoiMsg1Byte(ID_TRAIT_LIGNE_GENE, 2);
    #endif
}
void trait_Rayon_De_Courbure_Clotho(struct Ordre_deplacement* monDpl)
{
    unsigned long tc, ta, td;   //tc temps à vitesse constante, ta en acceleration, td en deceleration, temp en nombre d'étt d'automate
    double accel = 0;
    int pcons, rapport;
    
    #if F_DBUG_TRAIT_ETAT_CLOTHO
    //Serial.println("ID_TRAIT_CLOTHO");
    remplirStruct(ID_TRAIT_CLOTHO, 1, 0x01, 0,0,0,0,0,0,0, DATArobot);
    writeStructInCAN(DATArobot);                             //CAN
    //CANenvoiMsg1Byte(ID_TRAIT_CLOTHO, 1);
    #endif
    
    int flagVir = 1;
    maClotho.E = LARGEUR_ROBOT_TIC;
    
    maClotho.accel = monDpl->amax * 0.001;//accel;
    maClotho.vit = fabs(monDpl->vinit);
    maClotho.angleDArc = fabs((double)monDpl->theta_ray);
    
    
    maClotho.rayondDeCourbre = monDpl->rayon/DTIC; // on convertir des mm en tic
    
    flagVir = possibiliteVirage(&maClotho);
    remplirStruct(ID_CLOTHO_IMPOSSIBLE, 1, flagVir, 0,0,0,0,0,0,0, DATArobot);
    writeStructInCAN(DATArobot);                             //CAN
    //CANenvoiMsg1Byte(ID_CLOTHO_IMPOSSIBLE, flagVir);
        
    trajectoire(&maClotho);
    
    if(!flagVir && 0){
      DATArobot.ID = ID_CLOTHO_IMPOSSIBLE;
      DATArobot.RTR = true;
      writeStructInCAN(DATArobot);                             //CAN
      //CANenvoiMsg(ID_CLOTHO_IMPOSSIBLE);
    }
    
    #if F_DBUG_TEMPS_CALCUL_CLOTHO
    //CANenvoiMsg6x1Byte(ID_TEMPS_CALCUL_CLOTHO, tC1, tC2, tC3, tC4, tC5, nbexpr);
    #endif
   
    ta = (unsigned long)maClotho.tClotho;      //Calcul du temps d'acceleration
    tc = (unsigned long)maClotho.tArc;
    td = (unsigned long)maClotho.tClotho;
    monDpl->ta = maClotho.tClotho;      //Calcul du temps d'acceleration
    monDpl->tc = maClotho.tArc;
    monDpl->td = maClotho.tClotho;
    
    #if F_DBUG_CLOTHO_TRAIT_DOUBLE
    /*
    CANenvoiMsg8Bytes(ID_ENTRAXE, &maClotho.E);
    
    CANenvoiMsg8Bytes(ID_ALPHA, &maClotho.angleDArc);
    
    CANenvoiMsg8Bytes(ID_RAYON, &maClotho.rayondDeCourbre);
    
    CANenvoiMsg8Bytes(ID_VITESSE, &maClotho.vit);
    
    CANenvoiMsg8Bytes(ID_ACCELERATION, &maClotho.accel);
    
    CANenvoiMsg8Bytes(ID_TCLOTHO, &maClotho.tClotho);
    
    CANenvoiMsg8Bytes(ID_TARC, &maClotho.tArc);*/
        
    #endif
    
    
    #if F_DBUG_CLOTHO_TRAIT_TPS
        
        CANenvoiMsg2x4Bytes(ID_TEMPS_LONG_1, &ta, &tc);
        /*
        CANenvoiMsg4Bytes(ID_TEMPS_LONG_2, &td);
        CANenvoiMsg2x2Bytes(ID_DBUG_LIGNE_GENE_VIT, monDpl->vinit, monDpl->vinit); */
        
    #endif
    
    
    #if F_DBUG_TRAIT_ETAT_CLOTHO
    remplirStruct(ID_TRAIT_CLOTHO, 1, 2, 0,0,0,0,0,0,0, DATArobot);
    //CANenvoiMsg1Byte(ID_TRAIT_CLOTHO, 2);
    #endif
    
}
/***************************************************************************************
 NOM : Mouvement Elementaire_Gene                                                        
 ARGUMENT : long pcons -> distance a parcourir (>0 : avancer et <0 : reculer ) // exprimée en tic         
            short vmax -> vitesse maximale       // tic/TE                                     
            short amax -> acceleration maximale      // tic/(TE)²                                 
            short dmax -> deceleration maximale      // tic/(TE)²        
            short vinitb -> vitesse de départ       // tic/TE                                             
            short vfin -> vitesse d'arrivée       // tic/TE                                     
 RETOUR : rien                                                                        
 DESCRIPTIF : Fonction (automate)  appelee pour effectuer une ligne droite avec vitesse de départ et d'arrivée imposée                        
***************************************************************************************/
void Mouvement_Elementaire_Gene(struct Ordre_deplacement monDpl)//fait
{    
    #if F_DBUG_LIGNE_GEN
    mscount2 ++;
    if(mscount2 >= (1000/TE_100US))    
    {
        mscount2 = 0;
        CANenvoiMsg1x8Bytes(ID_VIT, &consigne_vit);
        CANenvoiMsg1x8Bytes(ID_POS, &consigne_pos);
    }
    #endif
    //Declaration des variables
    static unsigned long tc, ta, td;   //tc temps à vitesse constante, ta en acceleration, td en deceleration
    static long pcons;
    static short vinit, vfin, vmax, amax, dmax;
    static double accel = 0, decel = 0;
    
    static short memo_etat_automate = 0;
    static short defaut_asserv = 0;
    
    
    
    
    
    if(etat_automate_depl == INITIALISATION)
    {   //Etat d'initialisation et de calcul des variables
        //Remise a zero des variables car on effectue un nouveau deplacement NON si on est tjr en mouvement
        //consigne_pos = 0;
        
        if(monDpl.vinit == 0)//Remise a zero des variables car on effectue un nouveau deplacement
        {
            consigne_pos = 0;
            Asser_Init();
            roue_drt_init = lireCodeurD();
            roue_gch_init = lireCodeurG();
        }
        //consigne_vit = vinit;
        cpt = 0;
        //defaut_asserv = 0;
        
        accel = monDpl.amax * 0.001;
        decel = monDpl.dmax * 0.001;
            
        pcons = monDpl.distance;
        vinit = monDpl.vinit;
        vfin = monDpl.vfin;
        vmax = monDpl.vmax;
        amax = monDpl.amax;
        dmax = monDpl.dmax;
        
        
        ta = monDpl.ta;
        tc = monDpl.tc;
        td = monDpl.td;
        
        if(tc) etat_automate_depl = ACCELERATION_TRAPEZE;
        else etat_automate_depl = ACCELERATION_TRIANGLE;
        
        #if F_DBUG_LIGNE_GEN_TPS
        CANenvoiMsg2x4Bytes(ID_TEMPS_LONG_1, &ta, &tc);
        /*
        CANenvoiMsg1x4Bytes(ID_TEMPS_LONG_2, &td);                
        
        CANenvoiMsg2x2Bytes(ID_DBUG_LIGNE_GENE_VIT, vinit, vfin);
        */
        CANenvoiMsg2x4Bytes(ID_DIST_TIC_GENE, &pcons, &pcons);
        
        #endif
        
        
    }
       
    switch(etat_automate_depl)      //Automate de gestion du deplacement
    {
     case ACCELERATION_TRAPEZE :    //Etat d'acceleration en profil trapeze
        cpt ++;
        
        //Incrementation de la consigne de vitesse par la valeur de l'acceleration
        if (pcons>0) consigne_vit += accel;
        else consigne_vit -= accel;
        
        //Incrementation de la consigne de position
        consigne_pos += consigne_vit;
        
        if(cpt >= ta)   //Condition pour quitter la phase d'acceleration
        {
            etat_automate_depl = VITESSE_CONSTANTE_TRAPEZE;      //Passage a l'etat VITESSE_CONSTANTE
            cpt = 0;
            consigne_vit = vmax;
        }
        break;
        
     case VITESSE_CONSTANTE_TRAPEZE :   //Etat de vitesse constante en profil trapeze
        cpt ++; 
        //Incrementation de la consigne de position
        consigne_pos += consigne_vit;
        
        //Si il n'y a pas d'enchainements
        if(cpt >= tc)     //Condition pour quitter la phase de vitesse constante
        {
            etat_automate_depl = DECELERATION_TRAPEZE;      //Passage a l'etat DECELERATION
            cpt = 0;
        }
        break;
        
     case DECELERATION_TRAPEZE :    //Etat de deceleration en profil trapeze
        cpt ++;
         
        //Incrementation de la consigne de vitesse par la valeur de l'acceleration
        if (pcons>0) consigne_vit -= decel;
        else consigne_vit += decel;
        
        //Incrementation de la consigne de position
        consigne_pos += consigne_vit;
        
        if(cpt >= td)       //Condition pour quitter la phase de deceleration en profil trapeze
        {
                 //Passage a l'etat ARRET
            consigne_pos = pcons;
            consigne_vit = vfin;
            if(vfin == 0)etat_automate_depl = ARRET;
            else
            {
               finMvtElem = 1;
                etat_automate_depl = INITIALISATION;  
            }
            cpt = 0;
        }
        break;
        
     case ARRET :       //Etat d'arret
        cpt ++;
        
        if(cpt >= 20)       //Condition pour sortir de l'etat arret
        {
            finMvtElem = 1;
            etat_automate_depl = INITIALISATION;        //Passage a l'etat INITIALISATION
        }
        break;
        
    case ACCELERATION_TRIANGLE :        //Etat d'acceleration en profil triangle
        cpt ++;
        
        //Incrementation de la consigne de vitesse par la valeur de l'acceleration
        if (pcons>0) consigne_vit += accel;
        else consigne_vit -= accel;
        
        //Incrementation de la consigne de position
        consigne_pos += consigne_vit;
        
        if(cpt >= ta)   //Condition pour quitter la phase d'acceleration
        {
            etat_automate_depl = DECELERATION_TRIANGLE;      //Passage a l'etat VITESSE_CONSTANTE
            cpt = 0;
            consigne_vit = vmax;
        }
        break;
        
    case DECELERATION_TRIANGLE :        //Etat de deceleration en profil triangle
        cpt ++;
        
        //Incrementation de la consigne de vitesse par la valeur de l'acceleration
        if (pcons>0) consigne_vit -= decel;
        else consigne_vit += decel;
        
        //Incrementation de la consigne de position
        consigne_pos += consigne_vit;
        
        if(cpt >= td)       //Condition pour quitter la phase de deceleration en profil trapeze
        {
            consigne_pos = pcons;
            consigne_vit = vfin;
            if(vfin == 0)etat_automate_depl = ARRET;
            else
            {
               finMvtElem = 1;
                etat_automate_depl = INITIALISATION;  
            }
            cpt = 0;
        }
        break;
        
    case TROP_D_ERREUR_ASSERV :
        etat_automate_depl = memo_etat_automate;
        defaut_asserv++;
        break;
    
    
    default:
    break;
    }
    if(etat_automate_depl != INITIALISATION)
    {
        //Calcul des commandes
        double cmdD, cmdG, erreur;
        Asser_Pos_Mot(roue_gch_init + consigne_pos, roue_drt_init + consigne_pos, &cmdG, &cmdD);
        //cmdD = Asser_Pos_MotD(roue_drt_init + consigne_pos);
        //cmdG = Asser_Pos_MotG(roue_gch_init + consigne_pos);
        erreur = ErreurPosG;
        
/*        if (fabs(ErreurPosD - erreur) > EXPLOSION_TAUX) {
            // Trop d'écart
            memo_etat_automate = etat_automate_depl;
            if (defaut_asserv<3) {
                //etat_automate_depl = TROP_D_ERREUR_ASSERV;
                defaut_asserv ++;
            } else {
                defaut_asserv = 0;//msg defaut???
            }
        }*/
        //Ecriture du PWM sur chaque modeur
        write_PWMG(cmdG);   
        write_PWMD(cmdD);
        
        //Arret si le robot est bloqué
        lectureErreur();
    }
}
/***************************************************************************************
 NOM : Rayon_De_Courbure_Clotho                                                              
 ARGUMENT : short rayon -> rayon de l'arc de cercle a parcourir                       
            short theta -> angle à parcourir                      // 1/10 deg                    
            short vmax -> vitesse maximale                        //  tic/TE                    
            short dmax -> deceleration maximale                   //  tic/TE²                     
            short amax -> acceleration maximale                   //  tic/TE²                   
            short sens -> sens de deplacement                      // gauche droite                   
 RETOUR : rien                                                                        
 DESCRIPTIF : Fonction appelee pour effectuer un arc de cercle                        
              Dans un rayon de courbure, on avance en suivant un arc de cercle        
              situé à une distance rayon du centre du robot.                          
              On va donc faire comme avec une ligne droite, sauf qu'une des roues     
              (la roue intérieure dans le cercle) va moins avancer que l'autre        
              Le rapport de distance entre les 2 roues est :                          
              distance de la roue interieure par rapport au centre du cercle,         
              divisé par distance de la roue exterieur par rapport au centre,         
              Ou encore (2*(rayon - (largeur du robot/2))) / (2*(rayon + (largeur du robot/2)))  
              Ou encore (2*rayon - largeur) / (2*rayon + largeur)                     
              Pour le reste, c'est comme une ligne droite,                            
              Sauf que la distance à parcourir (pour la ext) est                      
              celle de l'arc de cercle : (rayon + largeur/2) * angle à parcourir      
***************************************************************************************/
void Rayon_De_Courbure_Clotho(struct Ordre_deplacement monDpl)//fait
{
    //Declaration des variables
    static double tc, ta, td, tarc;   //tc temps à vitesse constante, ta en acceleration, td en deceleration, temp en nombre d'étt d'automate
    static double vinit, accel = 0;
    static int sens;
    static short memo_etat_automate = 0;
    static short defaut_asserv = 0;
    int pcons, rapport;
    static double consigne_posv = 0, consigne_posl = 0;
    
    
    
    mscount2 ++;
    
    #if F_DBUG_CLOTHO_VIT
    if(mscount2 >= (106600/TE_100US))//envoyer des informations régulièrement
    {
        mscount2 = 0;
        CANenvoiMsg1x8Bytes(ID_VIT, &vinit);
        CANenvoiMsg1x8Bytes(ID_VIT1, &consigne_vit);
        CANenvoiMsg1x8Bytes(ID_POS, &consigne_posv);
        CANenvoiMsg1x8Bytes(ID_POS1, &consigne_posl);
    }
    
    #endif
    
    
    
    if(etat_automate_depl == INITIALISATION)
    {
        
        cpt = 0;
        consigne_posv = 0;
        consigne_posl = 0;
        
        accel = monDpl.amax* 0.001;
        
        consigne_vit = monDpl.vinit;
        vinit = monDpl.vinit;
       
        ta = monDpl.ta;      //Calcul du temps d'acceleration
        tc = monDpl.tc;// + ta;      //Calcul du temps constant
        td = monDpl.td;//+tc;
        
        sens = monDpl.sens;
        etat_automate_depl = ACCELERATION_TRAPEZE;
        
        
        #if F_DBUG_CLOTHO_TPS/*
            CANenvoiMsg1x8Bytes(ID_VITESSE, &vinit);
            CANenvoiMsg1x8Bytes(ID_ACCELERATION, &accel);
            CANenvoiMsg1x8Bytes(ID_TCLOTHO, &ta);
            CANenvoiMsg1x8Bytes(ID_TARC, &tc); */           
        
        #endif
    }
    
    
    switch(etat_automate_depl)      //Automate de gestion du deplacement
    {
        
        case ACCELERATION_TRAPEZE :    //Etat d'acceleration en profil trapeze
            cpt ++;
            
            //Incrementation de la consigne de vitesse par la valeur de l'acceleration
        
            if(vinit>0) consigne_vit -= accel;  
            else consigne_vit += accel;          
            
            //Incrementation de la consigne de position
            consigne_posv += vinit;
            consigne_posl += consigne_vit;
            
            if(cpt >= ta)   //Condition pour quitter la phase d'acceleration
            {
                etat_automate_depl = VITESSE_CONSTANTE_TRAPEZE;      //Passage a l'etat VITESSE_CONSTANTE
                cpt = 0;
                
                CANenvoiMsg1x8Bytes(ID_ENTRAXE, &vinit); 
                
                CANenvoiMsg1x8Bytes(ID_RAYON, &consigne_vit);
                
                //consigne_vit = vmax;
            }
            break;
            
        case VITESSE_CONSTANTE_TRAPEZE :   //Etat de vitesse constante en profil trapeze
            cpt ++; 
            //Incrementation de la consigne de position
            
            consigne_posv += vinit;
            consigne_posl += consigne_vit;
            
            //Si il n'y a pas d'enchainements
            if(cpt >= tc)     //Condition pour quitter la phase de vitesse constante
            {
                etat_automate_depl = DECELERATION_TRAPEZE;      //Passage a l'etat DECELERATION
                cpt = 0;
            }
            break;
            
        case DECELERATION_TRAPEZE :    //Etat de deceleration en profil trapeze
            cpt ++;
             
            //Incrementation de la consigne de vitesse par la valeur de l'acceleration
            if(vinit>0) consigne_vit += accel;          
            else consigne_vit -= accel;
            //Incrementation de la consigne de position
            
            consigne_posv += vinit;
            consigne_posl += consigne_vit;
            
            if(cpt >= td)       //Condition pour quitter la phase de deceleration en profil trapeze
            {
                consigne_pos = consigne_posv;
                consigne_vit = vinit;
                cpt = 0;
                finRayonCourbureClo = 1;
                etat_automate_depl = INITIALISATION; 
                //Serial.println("0x002");
                remplirStruct(0x002,0,0,0,0,0,0,0,0,0, DATArobot);
                writeStructInCAN(DATArobot);
                //CANenvoiMsg(0x002);
            }
            break;
            
            
            
        
        case TROP_D_ERREUR_ASSERV :
            etat_automate_depl = memo_etat_automate;
            defaut_asserv++;
            break;
        
        default:break;    
    }
    
    if (etat_automate_depl != INITIALISATION){
        double cmdD, cmdG;
        //Determination des commandes en fonction de la direction de deplacement du robot
        if(sens >= 0)  
        {
            
            Asser_Pos_Mot(roue_gch_init + consigne_posl, roue_drt_init + consigne_posv, &cmdG, &cmdD);
            //cmdD = Asser_Pos_MotD(roue_drt_init + consigne_posv);
            //cmdG = Asser_Pos_MotG(roue_gch_init + consigne_posl);
        }else{
            
            Asser_Pos_Mot(roue_gch_init + consigne_posv, roue_drt_init + consigne_posl, &cmdG, &cmdD);
            //cmdD = Asser_Pos_MotD(roue_drt_init + consigne_posl);
            //cmdG = Asser_Pos_MotG(roue_gch_init + consigne_posv);
        }
        
/*        if (fabs(ErreurPosD - ErreurPosG) > EXPLOSION_TAUX) {
            // Trop d'écart
            memo_etat_automate = etat_automate_depl;
            if (defaut_asserv<3) {
                etat_automate_depl = TROP_D_ERREUR_ASSERV;
            } else {
                defaut_asserv = 0;//msg defaut???
            }
        }*/
        
        //Envoi de la vitesse aux moteurs
        write_PWMG(cmdG);   
        write_PWMD(cmdD);
        //Arret si le robot est bloqué
        lectureErreur();
    }
}
/***************************************************************************************
 NOM : X_Y_Theta                                                                      
 ARGUMENT : long px -> position en x a atteindre                                      
            long py -> position en y a atteindre                                      
            long ptheta -> orientation en Theta a atteindre                           
            long sens -> sens de deplacement                                          
            short amax -> acceleration maximale                                       
            short dmax -> deceleration maximale                                       
 RETOUR : rien                                                                        
 DESCRIPTIF : Fonction appelee qui gère le deplace d'un point à un autre              
***************************************************************************************/
void X_Y_Theta(long px, long py, long ptheta, long sens, short vmax, short amax)//fait
{
    //Variables présentes Odo_x, Odo_y : position de départ
    //Declaration des variables
    static short dist = 0, ang1 = 0, ang2 = 0;
    short val[4], dmax;
    
    dmax = amax;
    
    switch(etat_automate_xytheta)
    {
        case INIT_X_Y_THETA :   //etat INIT_X_Y_THETA
            //Mode Avancer
            if(sens >= 0)
                {
                    // Son hypothénuse correspond à la distance à parcourir
                    dist = (short)sqrt((px - Odo_x)*(px - Odo_x)+(py - Odo_y)*(py - Odo_y));
                    
                    // la 1ere rotation correspond à l'angle du triangle, moins l'angle de la position de départ
                    // C'est-à-dire la tangente du côté opposé sur l'angle adjacentç
                    // La fonction atan2 fait le calcul de la tangente en radians, entre Pi et -Pi
                    // On rajoute des coefficients pour passer en degrés
                    // On ajoute 7200 dixièmes de degrés pour être sûrs que le résultat soit positif
                    //ang1 = (short)((atan2((double)(py - Odo_y), (double)(px - Odo_x)) * 1800 / PI) - Odo_theta + 7200) % 3600;
                    
                    if((((py-Odo_y)!=0)&&((px-Odo_x)!=0))||((px-Odo_x)!=0)){
                        ang1 = (short)((atan2((double)(py - Odo_y), (double)(px - Odo_x)) * 1800 / M_PI) - Odo_theta + 7200) % 3600;}
                        
                    // On passe le résultat entre -1800 et 1800
                    if(ang1 > 1800) {ang1 = (ang1 - 3600);}
                    
                    // La 2è rotation correspond à l'angle de destination, moins l'angle à la fin de la ligne droite,
                    // donc le même qu'à la fin de la 1ère rotation, donc l'angle de départ plus la première rotation
                    // On ajoute 3600 pour être sûr d'avoir un résultat positif
                    ang2 = (short)(ptheta - ang1 - Odo_theta + 3600) % 3600;
                    
                    // On passe le résultat entre -1800 et 1800
                    if(ang2 > 1800) ang2 = (ang2 - 3600);
                    
                    // On transforme les résultats en distance et angles utilisables avec les fonctions déjà définies
                    dist = dist * RESOLUTION_ROUE_CODEUSE / PERIMETRE_ROUE_CODEUSE;
                    ang1 = LARGEUR_ROBOT * M_PI * RESOLUTION_ROUE_CODEUSE * ang1 / (3600 * PERIMETRE_ROUE_CODEUSE);
                    ang2 = LARGEUR_ROBOT * M_PI * RESOLUTION_ROUE_CODEUSE * ang2 / (3600 * PERIMETRE_ROUE_CODEUSE);
                }
            
            //Mode Reculer
            else if(sens < 0)
            {
                // Idem qu'au-dessus, mais laligne droite doit être faite en marche arrière
                // La distance est l'opposé de celle calculée au dessus
                // Les angles sont les mêmes à 1800 près
                dist = -(short)sqrt((px-Odo_x)*(px-Odo_x)+(py-Odo_y)*(py-Odo_y));
               
                //Premiere rotation
                //ang1 = (short)(((atan2((double)(py - Odo_y), (double)(px - Odo_x)) * 1800 / PI) - Odo_theta) + 5400) % 3600;
                
                if((((py-Odo_y)!=0)&&((px-Odo_x)!=0))||((px-Odo_x)!=0))
                        ang1 = (short)((atan2((double)(py - Odo_y), (double)(px - Odo_x)) * 1800 / M_PI) - Odo_theta + 5400) % 3600;
                        
                if(ang1 > 1800) {ang1 = (ang1 - 3600);}
                
                //Deuxieme rotation
                ang2 = (short)(ptheta - ang1 - Odo_theta + 3600) % 3600;
                
                if(ang2 > 1800) {ang2 = (ang2 - 3600);}
                
                // On transforme les résultats en distance et angles utilisables avec les fonctions déjà définies
                dist = dist * RESOLUTION_ROUE_CODEUSE / PERIMETRE_ROUE_CODEUSE;
                ang1 = LARGEUR_ROBOT * M_PI * RESOLUTION_ROUE_CODEUSE * ang1 / (3600 * PERIMETRE_ROUE_CODEUSE);
                ang2 = LARGEUR_ROBOT * M_PI * RESOLUTION_ROUE_CODEUSE * ang2 / (3600 * PERIMETRE_ROUE_CODEUSE);
            }
            
            param_xytheta[0] = ang1;
            param_xytheta[1] = dist;
            param_xytheta[2] = ang2;
            
            // La fonction xythéta utilise un automate propre, similaire à l'automate général
            etat_automate_xytheta = ROTATION_X_Y_THETA_1;  //Passage a l'etat ROTATION_X_Y_THETA_1
            etat_automate_depl = INITIALISATION;
            
            break;
        
        case ROTATION_X_Y_THETA_1 : //etat ROTATION_X_Y_THETA_1
            //Execution de la fonction de rotation pour la fonction de deplacement X_Y_THETA
            Mouvement_Elementaire(ang1, vmax, amax, dmax, MOUVEMENT_ROTATION);
        
            if(finMvtElem)
            {
                Asser_Init();
                roue_drt_init = lireCodeurD();
                roue_gch_init = lireCodeurG();
                finMvtElem = 0;
                //Serial.println("INSTRUCTION_END_MOTEUR");
                remplirStruct(INSTRUCTION_END_MOTEUR, 2, 0x30, 0x00,0,0,0,0,0,0, DATArobot);
                writeStructInCAN(DATArobot);
                ////CANenvoiMsg2x1Byte(INSTRUCTION_END_MOTEUR, 0x30, 0);
                
                etat_automate_xytheta = LIGNE_DROITE_X_Y_THETA; //Passage a l'etat LIGNE_DROITE_X_Y_THETA
            }
            break;
        
        case LIGNE_DROITE_X_Y_THETA :   //etat LIGNE_DROITE_X_Y_THETA_1
            //Execution de la fonction de ligne droite pour la fonction de deplacement X_Y_THETA
            Mouvement_Elementaire(dist, vmax, amax, dmax, MOUVEMENT_LIGNE_DROITE);
            
            if(finMvtElem)
            {
                Asser_Init();
                roue_drt_init = lireCodeurD();
                roue_gch_init = lireCodeurG();
                finMvtElem = 0;
                //Serial.println("INSTRUCTION_END_MOTEUR");
                remplirStruct(INSTRUCTION_END_MOTEUR, 2, 0x40, 0x00,0,0,0,0,0,0, DATArobot);
                writeStructInCAN(DATArobot);
                ////CANenvoiMsg2x1Byte(INSTRUCTION_END_MOTEUR, 0x40, 0);
                
                etat_automate_xytheta = ROTATION_X_Y_THETA_2;   //Passage a l'etat ROTATION_X_Y_THETA_2
            }
            break;
        
        case ROTATION_X_Y_THETA_2 : //etat ROTATION_X_Y_THETA_2
            //Execution de la fonction de rotation pour la fonction de deplacement X_Y_THETA           
            Mouvement_Elementaire(ang2, vmax, amax, dmax, MOUVEMENT_ROTATION);
            
            if(finMvtElem)
            {
                Asser_Init();
                roue_drt_init = lireCodeurD();
                roue_gch_init = lireCodeurG();
                finMvtElem = 0;
                finXYT = 1;
                
                etat_automate_xytheta = INIT_X_Y_THETA; //Passage a l'etat INIT_X_Y_THETA
                
            }
            break;
        
        default :   //Etat par defaut, on fait la meme chose que dans l'etat d'initialisation
        etat_automate_xytheta = INIT_X_Y_THETA;
        break;
    }
}
/***************************************************************************************
 NOM : Recalage                                                                       
 ARGUMENT : long pcons -> distance a parcourir (>0 : avancer et <0 : reculer          
            short vmax -> vitesse maximale                                            
            short amax -> acceleration maximale                                       
            short dir -> coordonnée a recaler                                         
            short nv_val -> la valeur seuil                                           
 RETOUR : rien                                                                        
 DESCRIPTIF : Fonction appelee pour effectuer un recalage du robot                    
              Cette fonction sert à se coller à une bordure du terrain de jeu         
              jusqu'à y être totalement parallèle, puis elle met à jour les données   
              de position avec la position donnée dans le message CAN correspondant.  
              Cette fonction est sensiblement la même qu'une ligne droite             
              sauf que la vitesse est réduite tt on ne passe jamais en phase de       
              décélération. A la place, on vérifie l'erreur de position,              
              et si elle dépasse une certaine valeur sur les 2 roues,                 
              Ca veut dire qu'on est contre un obstacle et qu'on arrive plus à        
              reculer (d'où l'augmentation de l'erreur). Une fois qu'on a détecté     
              l'augmentation d'erreur sur les 2 roues, ça veut dire qu'on a fini      
              de se recaler sur l'obstacle                                            
***************************************************************************************/
void Recalage(int pcons, short vmax, short amax, short dir, short nv_val)//fait
{
    vmax = 5; //15
    amax = 5; //75
    
    //Mode AVANCER
    if(pcons >= 0)          //if(pcons >= 0)
    {
        switch(etat_automate_depl)
        {
            case INIT_RECALAGE :   //etat INIT_RECELAGE
                etat_automate_depl = ACCELERATION_RECALAGE;     //Passage a l'etat ACCELERATION_RECALAGE
                consigne_pos = 0;   //Initialisation des differentes variables
                cpt = 0;
                break;
             
            case ACCELERATION_RECALAGE :    //etat ACCELERATION_RECALAGE
                // Phase d'accéleration comme si on faisait une ligne droite
               
                if(consigne_pos >= pcons/2) 
                    etat_automate_depl = FIN_RECALAGE;  //Passage a l'etat FIN_RECALAGE 
                
                cpt ++;
                
                //Calcul de la consigne de position
                consigne_pos += (((double)cpt * (double)amax)/1000.0);
                
                if(cpt >= ((double)vmax / ((double)amax/1000.0)))
                {
                    etat_automate_depl = VITESSE_CONSTANTE_RECALAGE;  //Passage a l'etat VITESSE_CONSTANTE_RECALAGE
                }
                
                // Si les 2 erreurs sont supérieures à un seuil défini, on a fini le recalage
                if((ErreurPosD >= RECALAGE_TH) && (ErreurPosG >= RECALAGE_TH))
                {
                    etat_automate_depl = FIN_RECALAGE;  //Passage a l'etat FIN_RECALAGE
                    write_PWMD(0);
                    write_PWMG(0);
                }
                // Si on détecte une augmentation d'erreur sur une roue, on l'arrête pour éviter de patiner
                else if(ErreurPosD >= RECALAGE_TH)  write_PWMD(0);
                else if(ErreurPosG >= RECALAGE_TH)  write_PWMG(0);
                break;
            
            case VITESSE_CONSTANTE_RECALAGE :   //etat VITESSE_CONSTANTE_RECALAGE
                // Phase de vitesse constante
                consigne_pos += vmax;
                
                // Idem que plus haut, on surveille les erreurs des 2 roues
                if((ErreurPosD >= RECALAGE_TH) && (ErreurPosG >= RECALAGE_TH))
                {
                    etat_automate_depl = FIN_RECALAGE;  //Passage a l'etat FIN_RECALAGE
                    write_PWMD(0);
                    write_PWMG(0);
                }
                else if(ErreurPosD >= RECALAGE_TH) write_PWMD(0);  //Mise a zero de la commande 1
                else if(ErreurPosG >= RECALAGE_TH) write_PWMG(0);  //Mise a zero de la commande 2
                break;
                
            case FIN_RECALAGE :     //etat FIN_RECALAGE
                // Fin du recalage, on met à jour les données de position
                if(cpt >=20)
                {
                    //Recalage de x
                    if(dir == 1)
                    {
                        Odo_x = nv_val;
                        // On met l'angle à jour en fonction de la position sur le terrain
                        // Si on est dans la partie haute ( > la moitié), on est dans un sens,
                        // Si on est dans la partie basse, on met à jour dans l'autre sens
                        // On prend aussi en compte le sens dans lequel on a fait la ligne droite
                        
                        if(nv_val > 1000)
                        {
                            if(pcons >=0)
                                Odo_theta = 0;
                            else
                                Odo_theta = 1800;
                        }
                        
                        else
                        {
                            if(pcons >= 0)
                                Odo_theta = 1800;
                            else
                                Odo_theta = 0;
                        }
                    }
                    
                    //Recalage de y
                    else
                    {
                        Odo_y = nv_val;
                        
                        if(nv_val > 1500)
                        {
                            if(pcons >=0)
                                Odo_theta = 900;
                            else
                                Odo_theta = -900;
                        }
                        
                        else
                        {
                            if(pcons >= 0)
                                Odo_theta = -900;
                            else
                                Odo_theta = 900;
                        }
                    }
                    
                    etat_automate_depl = INIT_RECALAGE;
                    
                    finRecalage = 1;
                    
                    //Mise a zero des moteurs;
                    write_PWMD(0);
                    write_PWMG(0);
                }
                cpt ++;
                break;
        }
    }
    
    //Mode RECULER
    else if(pcons < 0)          //else if(pcons < 0)
    {
        switch(etat_automate_depl)
        {
            case INIT_RECALAGE :    //etat INIT_RECELAGE
                etat_automate_depl = ACCELERATION_RECALAGE;     //Passage a l'etat ACCELERATION_RECALAGE
                consigne_pos = 0;   //Initialisation des différentes variables
                cpt = 0;
                break;
            
            case ACCELERATION_RECALAGE :    //etat ACCELERATION_RECALAGE
                // Phase d'accéleration comme si on faisait une ligne droite
                if(consigne_pos <= pcons/2) etat_automate_depl = FIN_RECALAGE;  //Passage a l'etat FIN_RECALAGE
                
                cpt ++;
                consigne_pos -= (double)(((double)cpt * (double)amax)/1000.0);
                
                if(cpt >= ((double)vmax / ((double)amax/1000.0)))
                {
                    etat_automate_depl = VITESSE_CONSTANTE_RECALAGE;    //Passage a l'etat VITESSE_CONSTANTE_RECALAGE
                }
                
                // Si les 2 erreurs sont inférieures à un seuil défini, on a fini le recalage
                if(ErreurPosD <= -RECALAGE_TH && ErreurPosG <= -RECALAGE_TH)
                {
                    etat_automate_depl = FIN_RECALAGE;  //Passage a l'etat FIN_RECALAGE
                }
                // Si on détecte une augmentation d'erreur sur une roue, on l'arrête pour éviter de patiner
                else if(ErreurPosD <= -RECALAGE_TH) write_PWMD(0);
                else if(ErreurPosG <= -RECALAGE_TH) write_PWMG(0);
                break;
                
            case VITESSE_CONSTANTE_RECALAGE :   //etat VITESSE_CONSTANTE_RECALAGE
                // Phase de vitesse constante
                consigne_pos -= vmax;
                
                // Idem que plus haut, on surveille les erreurs des 2 roues
                if(ErreurPosD <= -RECALAGE_TH && ErreurPosG <= -RECALAGE_TH)
                {
                    etat_automate_depl = FIN_RECALAGE;  //Passage a l'etat FIN_RECALAGE
                }
                else if(ErreurPosD <= -RECALAGE_TH) write_PWMD(0);
                else if(ErreurPosG <= -RECALAGE_TH) write_PWMG(0);
                break;
                
            case FIN_RECALAGE :     //etat FIN_RECALAGE
                if(cpt >=20)
                {
                    //Recalage de x
                    if(dir == 1)
                    {
                        Odo_x = nv_val;
                        
                        if(nv_val > 1000)
                        {
                            if(pcons >=0)
                                Odo_theta = 0;
                            else
                                Odo_theta = 1800;
                        }
                        
                        else
                        {
                            if(pcons >= 0)
                                Odo_theta = 1800;
                            else
                                Odo_theta = 0;
                        }
                    }
                    
                    //Recalage de y
                    else
                    {
                        Odo_y = nv_val;
                       
                        if(nv_val > 1500)
                        {
                            if(pcons >=0)
                                Odo_theta = 900;
                            else
                                Odo_theta = -900;
                        }
                       
                        else
                        {
                            if(pcons >= 0)
                                Odo_theta = -900;
                            else
                                Odo_theta = 900;
                        }
                    }
                    
                    etat_automate_depl = 0;
                    
                    finRecalage = 1;
                    
                    //Mise a zero des moteurs
                    write_PWMD(0);
                    write_PWMG(0);
                }
                cpt ++;
                break;
        }
    }
    //Calcul des commandes
    double cmdD, cmdG;
    cmdD = Asser_Pos_MotD(roue_drt_init + consigne_pos);
    cmdG = Asser_Pos_MotG(roue_gch_init + consigne_pos);
    //Ecriture du PWM sur chaque modeur
    write_PWMG(cmdG);   
    write_PWMD(cmdD);
}
int Courbe_bezier(double distanceG, double distanceD)
{
    if((distanceG >= (666/FACTEUR_DIVISION)) && (distanceD >= (666/FACTEUR_DIVISION))) //Valeurs indiquant la fin de la courbe
    {
        consigne_posD = 0;
        consigne_posG = 0;
        return 1; //Retourne 1 pour indiquer la fin et passer au mouvement suivant
    }
    consigne_posG += distanceG;
    consigne_posD += distanceD;
   
    double cmdD, cmdG;
        
    cmdD = Asser_Pos_MotD(roue_drt_init + consigne_posD);
    cmdG = Asser_Pos_MotG(roue_gch_init + consigne_posG);
    write_PWMG(cmdG);   
    write_PWMD(cmdD);
    return 0;
}
/***************************************************************************************
 NOM : Odometrie                                                                      
 ARGUMENT : rien                                                                      
 RETOUR : rien                                                                        
 DESCRIPTIF : Fonction qui calcul la position et l'orientation du robot               
***************************************************************************************/
void Odometrie(void)//fait
{
    
    //Declaration des variables
    double dist, ang;   //distance, angle
    
    //Recuperation des valeurs des compteurs des encodeurs  
    Odo_val_pos_D = lireCodeurD();
    Odo_val_pos_G = lireCodeurG();
    erreur = Odo_val_pos_D - Odo_val_pos_G;
    //Calcul de la distance parcourue
    dist = 0.5*((Odo_val_pos_D - Odo_last_val_pos_D) + (Odo_val_pos_G - Odo_last_val_pos_G));
      
    //Calcul de la valeur de l'angle parcouru
    ang = ((Odo_val_pos_D - Odo_last_val_pos_D) -(Odo_val_pos_G - Odo_last_val_pos_G))*1800.0*PERIMETRE_ROUE_CODEUSE/(LARGEUR_ROBOT*M_PI*RESOLUTION_ROUE_CODEUSE);
    
    //Determination de la position sur le terrain en X, Y, Theta
    Odo_theta +=  ang;
    Odo_x += dist*cos((double)(Odo_theta*M_PI/1800.0))*PERIMETRE_ROUE_CODEUSE/RESOLUTION_ROUE_CODEUSE;
    Odo_y += dist*sin((double)(Odo_theta*M_PI/1800.0))*PERIMETRE_ROUE_CODEUSE/RESOLUTION_ROUE_CODEUSE;
    
    //Stockage de la derniere valeur de l'odometrie
    Odo_last_val_pos_D = Odo_val_pos_D;
    Odo_last_val_pos_G = Odo_val_pos_G;
    
    
    mscount1 ++;    
    //Condition d'envoi des informations de l'odometrie par CAN 
    if(mscount1 >= (500/TE_100US))//toutes les 50ms
    {
        digitalWrite(27, set);//pour mesurer le temps de mscount1 avec l'oscilloscope
        set = !set;
        mscount1 = 0;
        //Serial.println();
        //Serial.printf("Codeur D : %lf ; Codeur G : %lf erreur : %lf\n", Odo_val_pos_D, Odo_val_pos_G, erreur);

        CANenvoiMsg3x2Bytes(ODOMETRIE_SMALL_POSITION, Odo_x, Odo_y, ((int16_t)Odo_theta) % 3600);
        //CANenvoiMsg3x2Bytes(ODOMETRIE_SMALL_POSITION, Odo_x, Odo_y, ((int16)Odo_theta) % 3600);
        //CANenvoiMsg3x2Bytes(DEBUG_ASSERV, QuadDec_D_GetCounter(), QuadDec_G_GetCounter(), consigne_pos);
    }
    
        
    
}  
void Moteur_Init(){
  //init pins carte de puiss
  pinMode(inApin_MOTG, OUTPUT);
  pinMode(inApin_MOTD, OUTPUT);
  pinMode(inBpin_MOTG, OUTPUT);
  pinMode(inBpin_MOTD, OUTPUT);
  pinMode(PWM_MOTG, OUTPUT);
  pinMode(PWM_MOTD, OUTPUT);
  pinMode(27, OUTPUT);
  //mise à l'arret
  digitalWrite(inApin_MOTG, LOW);
  digitalWrite(inApin_MOTD, LOW);
  digitalWrite(inBpin_MOTG, LOW);
  digitalWrite(inBpin_MOTD, LOW);
  //init pwm
  setupPWM(PWM_MOTD, PWMDChannel);
  setupPWM(PWM_MOTG, PWMGChannel);
}
//----------------------------------------------------------------------fonctions CAN et BLE

void setupCAN(){
  while (!Serial);
  Serial.println("Base Roulante de Anas Le bg/dg");
  // start the CAN bus at 1000 kbps
  if (!CAN.begin(1000E3)) { //ici, nous avons modifié la bibliotheque CAN pour qu'elle soit compatible avec l'ESP32-E
    Serial.println("Starting CAN failed!");
    while (1);
  }
  CAN.onReceive(canReadData); //init CAN callback function
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
  Serial.println(" done");
  Serial.println();*/
}
void canReadData(int packetSize){
  remplirStruct(0,0,0,0,0,0,0,0,0,0, DATAtoControl);
  DATAtoControl.ID = CAN.packetId();
  DATAtoControl.ln = CAN.packetDlc();
  //Serial.printf("Received CAN, ID : 0x%.3X ; len : %d\n", DATAtoControl.ID, DATAtoControl.ln);
  // only print packet DATAtoControl.dt for non-RTR packets
  int i = 0;
  while (CAN.available())
  {
    DATAtoControl.dt[i]=CAN.read();
    i++;
  }
  canAvailable = true;
}
void canReadExtRtr(){
  if (CAN.packetExtended()){
      //Serial.print("extended ");
      DATAtoControl.extented = true;
  }else{DATAtoControl.extented = false;}
  if (CAN.packetRtr()){
      // Remote transmission request, packet contains no DATAtoControl.dt
      //Serial.print("RTR ");
      DATAtoControl.RTR = true;
  }
  else{DATAtoControl.RTR = false;}
  printCANMsg(DATAtoControl);
}
void readDATA(std::string contenuBT, CANMessage &theDATA){

  theDATA.extented = contenuBT[0];
  theDATA.RTR = contenuBT[1];
  theDATA.ID = contenuBT[4] + (contenuBT[5]<<8) + (contenuBT[6]<<16) + (contenuBT[7]<<24);
  theDATA.ln = contenuBT[8];
  int i;
  for(i=0;i<8;i++)
  {
    theDATA.dt[i]=contenuBT[i+9];
  }
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

void remplirStruct(int idf, char lenf, char dt0f, char dt1f, char dt2f, char dt3f, char dt4f, char dt5f, char dt6f, char dt7f, CANMessage &theDATA){
  theDATA.RTR = false;
  if(idf>0x7FF){theDATA.extented = true;}
  else{theDATA.extented = false;}
  theDATA.ID = idf;
  theDATA.ln = idf;
  theDATA.dt[0] = dt0f;
  theDATA.dt[1] = dt1f;
  theDATA.dt[2] = dt2f;
  theDATA.dt[3] = dt3f;
  theDATA.dt[4] = dt4f;
  theDATA.dt[5] = dt5f;
  theDATA.dt[6] = dt6f;
  theDATA.dt[7] = dt7f;
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
void remplirStruct2x4Bytes(uint32_t id, void *pdata1, void *pdata2){
    DATArobot.RTR = false;
    if(id>0x7FF){DATArobot.extented = true;}
    else{DATArobot.extented = false;}
    DATArobot.ID = id;
    DATArobot.ln = 8;
    memcpy(&DATArobot.dt, pdata1, 4);
    memcpy(&(DATArobot.dt[4]), pdata2, 4);
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
void masterBTConnect(std::string name){
  // Create the BLE Device
	Serial.println("Create the BLE Device..");
	BLEDevice::init("BaseRoulante");

	// Create the BLE Server
	Serial.println("Create the BLE Server..");
	pServer = BLEDevice::createServer();
	pServer->setCallbacks(new MyServerCallbacks());

	// Create the BLE Service
	BLEService *pService = pServer->createService(SERVICE_UUID);

	// Create a BLE Characteristic
	pTxCharacteristic = pService->createCharacteristic(
										  CHARACTERISTIC_UUID_TX,
		                                  BLECharacteristic::PROPERTY_NOTIFY);
                      
	pTxCharacteristic->addDescriptor(new BLE2902());

	BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
											   CHARACTERISTIC_UUID_RX,
		                                       BLECharacteristic::PROPERTY_WRITE);

	pRxCharacteristic->setCallbacks(new MyCallbacks());

	// Start the service
	pService->start();

	// Start advertising
	//pServer->getAdvertising()->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();

	Serial.println("Waiting a client connection to notify...");
}
void BtActualise(){
    // disconnecting
	if(!deviceConnected && oldDeviceConnected) {
		pServer->startAdvertising();   // restart advertising
		Serial.println("start advertising");
		oldDeviceConnected = deviceConnected;
	}
	// connecting
	if(deviceConnected && !oldDeviceConnected) {
		// do stuff here on connecting
        oldDeviceConnected = deviceConnected;
	}
  
}
//----------------------------------------------------------------------autres fonctions
/****************************************************************************************/
/* NOM : setupPWM                                                                       */
/* ARGUMENT : Pins de sortie et channel des PWM                                         */
/* RETOUR : rien                                                                        */
/* DESCRIPTIF : permet de setup les canaux de PWM et de les faire sortir sur les pins   */
/****************************************************************************************/
void setupPWM(int PWMpin, int PWMChannel)
{
  int freqMot = 20000;
  int resolution = 11;
  ledcSetup(PWMDChannel, freqMot, resolution);
  ledcSetup(PWMGChannel, freqMot, resolution);
  ledcAttachPin(PWM_MOTD, PWMDChannel);
  ledcAttachPin(PWM_MOTG, PWMGChannel);
}
void test_accel(void)//fonctionne
{
    static int etat_test_accel = 0;
    static double posG, posD, lposG, lposD;
    double accel_test;
    static int cpt_test_accel = 0;
    switch(etat_test_accel)
    {
    case 0:
        write_PWMD(8*250/10);
        write_PWMG(8*250/10);
        etat_test_accel = 1;
        break;
        
    case 1:
        posD = lireCodeurD();      //Recuperation de la valeur du compteur incrémental
        posG = lireCodeurG();      //Recuperation de la valeur du compteur incrémental
        
        accel_test = posG - lposG;
        //Serial.printf("accel test : %lf\n", accel_test);
        DATArobot.ID = ID_TEST_VITESSE;
        DATArobot.dt[0] = accel_test;
        DATArobot.ln = 1;
       // writeStructInCAN(DATArobot); 
        
        
        lposD = posD;
        lposG = posG;
        
        cpt_test_accel++;
        if(cpt_test_accel > 160){
        etat_test_accel = 2;
        }
        break;
        
    case 2:
        write_PWMD(8*250/10);
        write_PWMG(8*250/10);
        etat_test_accel = 3;
        break;
        
    case 3:
        posD = lireCodeurD();      //Recuperation de la valeur du compteur incrémental
        posG = lireCodeurG();      //Recuperation de la valeur du compteur incrémental
        
        accel_test = posG - lposG;
        //Serial.printf("accel test : %lf\n", accel_test);
        DATArobot.ID = ID_TEST_VITESSE;
        DATArobot.dt[0] = accel_test;
        DATArobot.ln = 1;
       // writeStructInCAN(DATArobot); 
        
        
        lposD = posD;
        lposG = posG;
        
        break;
    }
}

void onTime() {//fonction s'exécutent à chaque interruptions 
   mscount++;
   
}

void init_Timer(){
    // Configure le Prescaler a 80 le quartz de l ESP32 est cadence a 80Mhz => à vérifier pour l'esp32-32E, peut etre 40Mhz?
   // 80000000 / 80 = 1000000 tics / seconde
   timer = timerBegin(idTimer, prescaler, flag);
   timerAttachInterrupt(timer, &onTime, true);//fait qu'on execute la fonction onTime à chaque interruptions
    
   // Regle le declenchement d une alarme chaque seconde
   timerAlarmWrite(timer, 1, true);      //freq de 250 000 Hz    
   timerAlarmEnable(timer); //active l'alarme
}

void init_coef(){
    double k = TE/0.02;
    VMAX = Vmax_coef*k;       // 50       petit 600   gros 600
    AMAX = Amax_coef*k*k;    //  500     petit 6000  gros 6000
    AMAX_CLO = Ama_clo_coef*k*k;    //  500     petit 6000  gros 6000
    DMAX = Dmax_coef*0.75*k*k;   //  500    petit 6000   gros 6000
    liste = (struct Ordre_deplacement){TYPE_DEPLACEMENT_IMMOBILE,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
}