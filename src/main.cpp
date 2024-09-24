//----------------------------------------------------------------------Bibliotheques
#include <Arduino.h>
#include <espCan.h>
#include <CRAC_utility.h>
#include <ident_crac.h>
#include <buffer_circulaire.h>
#include "math.h"
#include <mouvement.h>
#include <timerAsserBas.h>
#include <moteur.h>


 

//----------------------------------------------------------------------Variables

volatile uint16_t        mscount = 0,      // Compteur utilisé pour envoyer échantillonner les positions et faire l'asservissement
                         mscount1 = 0,     // Compteur utilisé pour envoyer échantillonner les positions et faire l'asservissement
                         mscount2 = 0;     // Compteur utilisé pour envoyer la trame CAN d'odométrie

extern double Odo_val_pos_D, Odo_val_pos_G, Odo_last_val_pos_D, Odo_last_val_pos_G;

int nbValeurs = 0;

double Kp =8    , Ki =0.4, Kd =10.0;
                
unsigned short cpt = 0; int cpt_ordre = 0; //Va savoir à quoi ça sert        

bool set = false;
 
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

//----------------------------------------------------------------------prototypes fonctions 

//Fonctions principales : 
void calcul(void);
void CANloop();
void Odometrie(void);


//----------------------------------------------------------------------SETUP
void setup() {
  Serial.begin(921600);
  init_coef();
  setupCAN();
  
  Encodeur_Init(36, 39, 23, 22); Serial.printf("fin encodeur init\n");
  
  Moteur_Init(inApin_MOTD, inApin_MOTG, inBpin_MOTD, inBpin_MOTG, PWMD_Channel, PWMG_Channel); Serial.printf("fin moteur init\n");
  
  AsserInitCoefs(Kp, Ki, Kd);
  Asser_Init();
  //Interruption : le programme est cadence grâce a l'interrutpion du Timer
  init_Timer(); Serial.printf("fin init Timer\n");
  
  remplirStruct(DATArobot, ALIVE_MOTEUR,0,0,0,0,0,0,0,0,0);
  writeStructInCAN(DATArobot); Serial.printf("envoie CAN ALIVE_MOTEUR\n");
  
  Serial.println("fin setup\n");
  
  /*liste.type = TYPE_DEPLACEMENT_LIGNE_DROITE;//test
  liste.distance = 1000;*/
  mscount = 0;
}
//----------------------------------------------------------------------loop
void loop() {
    if(TempsEchantionnage(TE_100US)){
        CANloop();
        calcul();
        Odometrie(); 
    }
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
                };La fonction est la boucle principale qui gère la robotique de l'application. 
                Elle est appelée périodiquement et s'assure que la liste des ordres est remplie par l'ISR CAN. 
                Cette liste est basée sur la structure "Ordre_deplacement". 
                La fonction détermine le prochain mouvement à effectuer à partir de la liste des ordres, 
                et appelle la fonction correspondante pour l'exécuter.
                Le mouvement à effectuer est basé sur le type de mouvement indiqué dans la structure "Ordre_deplacement". 
                Le mouvement peut être une ligne droite, une rotation, un mouvement sur les axes x, y et theta, ou un mouvement 
                sur un rayon de courbure. La fonction utilise également une série de commutateurs pour exécuter les mouvements 
                et les contrôler en temps réel. Le premier commutateur est destiné à l'état précédent et compare l'état actuel 
                à l'état précédent pour déterminer si un changement d'état s'est produit. Le deuxième commutateur est destiné à 
                l'automate de déplacement précédent et utilise un état pour savoir si le robot est en mouvement ou non. 
                La fonction arrête le robot lorsque le mouvement suivant est "TYPE_END_GAME", lorsque le robot n'est pas asservi 
                ou lorsque le match est terminé. En outre, la fonction envoie des messages de débogage à l'aide de CANenvoiMsg1Byte(), 
                qui envoie un message CAN d'une longueur de 1 octet. Les messages de débogage sont destinés à fournir des informations 
                de débogage pour le débogage en temps réel de l'application. (Merci ChatGPT)
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
            
            //Serial.println("ID_DBUG_ETAT");
            remplirStruct(DATArobot,ID_DBUG_ETAT, 1, etat_prec, 0,0,0,0,0,0,0);
            writeStructInCAN(DATArobot);                             //CAN
            #if F_DBUG_ETAT_DPL
            remplirStruct(DATArobot,ID_DBUG_ETAT_DPL, 1, etat_automate_depl, 0,0,0,0,0,0,0);
            writeStructInCAN(DATArobot);                             //CAN
            #endif
        }
        #endif
        
        #if F_DBUG_ETAT_DPL
        if(etat_automate_depl_prec != etat_automate_depl && etat_automate_depl != 8)
        {
            etat_automate_depl_prec = etat_automate_depl;
            ////Serial.println("ID_DBUG_ETAT_DPL");
            remplirStruct(DATArobot,ID_DBUG_ETAT_DPL, 1, etat_automate_depl_prec, 0,0,0,0,0,0,0);
            writeStructInCAN(DATArobot);                             //CAN           
        }
       #endif
       
        if (Fin_Match){
            liste.type = (TYPE_END_GAME);
            ////Serial.println("INSTRUCTION_END_MOTEUR");
            //On prévient qu'on s'est arrêté
            // le dlc original était de 2 avec un 0 en second octet
            remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 1, 0x04, 0,0,0,0,0,0,0);
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
                remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 2, (Message_Fin_Mouvement&0xFF), ((Message_Fin_Mouvement>>8)&0xFF),0,0,0,0,0,0);
                writeStructInCAN(DATArobot);

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
                remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 2, (Message_Fin_Mouvement&0xFF), ((Message_Fin_Mouvement>>8)&0xFF),0,0,0,0,0,0);
                writeStructInCAN(DATArobot);
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
                remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 2, (Message_Fin_Mouvement&0xFF), ((Message_Fin_Mouvement>>8)&0xFF),0,0,0,0,0,0);
                writeStructInCAN(DATArobot);

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
                remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 2, (Message_Fin_Mouvement&0xFF), ((Message_Fin_Mouvement>>8)&0xFF),0,0,0,0,0,0);
                writeStructInCAN(DATArobot);
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
                ////Serial.println("ID_TRAIT");
                remplirStruct(DATArobot,ID_TRAIT, 1, 0x01, 0,0,0,0,0,0,0);
                writeStructInCAN(DATArobot);                             //CAN
                //CANenvoiMsg1Byte(ID_TRAIT, 1);
                #endif
                
                int compteurMvnt = 0;
                for(compteurMvnt = 0; compteurMvnt<nb_ordres; compteurMvnt++)
                {
                    #if (F_DBUG_TRAIT_ETAT)
                    ////Serial.println("ID_TRAIT");
                    remplirStruct(DATArobot,ID_TRAIT, 2, 0x03, compteurMvnt,0,0,0,0,0,0);
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
                ////Serial.println("ID_TRAIT");
                remplirStruct(DATArobot,ID_TRAIT, 1, 0x02, 0,0,0,0,0,0,0);
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
                remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 2, (Message_Fin_Mouvement&0xFF), ((Message_Fin_Mouvement>>8)&0xFF),0,0,0,0,0,0);
                writeStructInCAN(DATArobot);
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
                remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 2, (Message_Fin_Mouvement&0xFF), ((Message_Fin_Mouvement>>8)&0xFF),0,0,0,0,0,0);
                writeStructInCAN(DATArobot);
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
                remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 2, ASSERVISSEMENT_RECALAGE, 0,0,0,0,0,0,0);
                writeStructInCAN(DATArobot);
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
                ////Serial.println("ACKNOWLEDGE_BEZIER");
                remplirStruct(DATArobot,ACKNOWLEDGE_BEZIER,0,0,0,0,0,0,0,0,0);
                writeStructInCAN(DATArobot);
                //CANenvoiMsg(ACKNOWLEDGE_BEZIER);
            }
            
            nbValeurs ++; //Permet de savoir le nombre de valeurs utilisées
            if(Courbe_bezier(distanceG/(FACTEUR_DIVISION), distanceD/(FACTEUR_DIVISION))) //Vrai si fin du mouvement
            {
                liste.type = (TYPE_MOUVEMENT_SUIVANT);
                flagDebutBezier = 0;
                remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 2, (Message_Fin_Mouvement&0xFF), ((Message_Fin_Mouvement>>8)&0xFF),0,0,0,0,0,0);
                writeStructInCAN(DATArobot);
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
            // Arret_Brutal();
            //cpt_stop = 0;
            //etat_automate_depl = DECELERATION_TRAPEZE;
            //cpt=global_ta_stop;
            Message_Fin_Mouvement = ASSERVISSEMENT_STOP;
            liste.type = TYPE_STOP;
            remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 2, (Message_Fin_Mouvement&0xFF), ((Message_Fin_Mouvement>>8)&0xFF),0,0,0,0,0,0);
            writeStructInCAN(DATArobot);
            break;
            }
            case TYPE_STOP :{
        /*    cpt_stop ++;
            if ((cpt_stop > 25)))
            {
                if(stop_receive==0)
                    liste.type = TYPE_MOUVEMENT_SUIVANT;
            }
       */     break;
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
                roue_drt_init = lireCodeurD();//Ou on pourrait remettre à zero le compteur
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
    static signed char FIFO_lecture=0,FIFO_occupation=0,FIFO_max_occupation=0;

    FIFO_occupation=FIFO_ecriture-FIFO_lecture;
    if(FIFO_occupation<0){FIFO_occupation=FIFO_occupation+SIZE_FIFO;}
    if(FIFO_max_occupation<FIFO_occupation){FIFO_max_occupation=FIFO_occupation;}

/*   //if(canAvailable || BtAvailable){
    // if(canAvailable){canReadExtRtr();}//On le me ici pour ne pas surcharger l'interruption CAN.onRecveive
    // canAvailable = false; BtAvailable = false;
    ////Serial.println("CAN received");*/
    if(!FIFO_occupation){return;}
    switch (rxMsg[FIFO_lecture].ID)
    {

            case ESP32_RESTART:
                //Serial.println("ESP32_RESTART");
                esp_restart();
                
                break;
            case ASSERVISSEMENT_REQUETE_PID:
                //Serial.println("ASSERVISSEMENT_REQUETE_PID");
                CANenvoiMsg1x8Bytes(ASSERVISSEMENT_CONFIG_KPP, &KppD);
                CANenvoiMsg1x8Bytes(ASSERVISSEMENT_CONFIG_KPI, &KipD);
                CANenvoiMsg1x8Bytes(ASSERVISSEMENT_CONFIG_KPD, &KdpD);
                break;
            case ASSERVISSEMENT_CONFIG_KPP_DROITE:
                memcpy(&KppD, rxMsg[FIFO_lecture].dt, 8);
                KppDa = KppD;
                ////Serial.println("ASSERVISSEMENT_CONFIG_KPP_DROITE");
                break;
            case ASSERVISSEMENT_CONFIG_KPI_DROITE:
                memcpy(&KipD, rxMsg[FIFO_lecture].dt, 8);
                KipDa = KipD;
                ////Serial.println("ASSERVISSEMENT_CONFIG_KPI_DROITE");
                break;
            case ASSERVISSEMENT_CONFIG_KPD_DROITE:
                memcpy(&KdpD, rxMsg[FIFO_lecture].dt, 8);
                KdpDa = KdpD;
                ////Serial.println("ASSERVISSEMENT_CONFIG_KPD_DROITE");
                break;
                
            case ASSERVISSEMENT_CONFIG_KPP_GAUCHE:
                memcpy(&KppG, rxMsg[FIFO_lecture].dt, 8);
                KppGa = KppG;
                ////Serial.println("ASSERVISSEMENT_CONFIG_KPP_GAUCHE");
                break;
            case ASSERVISSEMENT_CONFIG_KPI_GAUCHE :
                memcpy(&KipG, rxMsg[FIFO_lecture].dt, 8);
                KipGa = KipG;
                ////Serial.println("ASSERVISSEMENT_CONFIG_KPI_GAUCHE");
                break;
            case ASSERVISSEMENT_CONFIG_KPD_GAUCHE :
                memcpy(&KdpG, rxMsg[FIFO_lecture].dt, 8);
                KdpGa = KdpG;
                ////Serial.println("ASSERVISSEMENT_CONFIG_KPD_GAUCHE");
                break;
                
            case ASSERVISSEMENT_CONFIG_KPP:{
                Kp = 0;
                memcpy(&Kp, rxMsg[FIFO_lecture].dt, 8);
                
                AsserInitCoefs(Kp, Ki, Kd);
                Serial.print("  ASSERVISSEMENT_CONFIG_KPP : ");
                Serial.printf("%f ", Kp);
                //Serial.println();
                }
                break;
            case ASSERVISSEMENT_CONFIG_KPI :
                Ki = 0;
                memcpy(&Ki, rxMsg[FIFO_lecture].dt, 8);
                AsserInitCoefs(Kp, Ki, Kd);
                Serial.print("  ASSERVISSEMENT_CONFIG_KPI : ");
                Serial.printf("%f ", Ki);
                //Serial.println();
                break;
            case ASSERVISSEMENT_CONFIG_KPD :
                Kd = 0;
                memcpy(&Kd, rxMsg[FIFO_lecture].dt, 8);
                AsserInitCoefs(Kp, Ki, Kd); 
                Serial.print("  ASSERVISSEMENT_CONFIG_KPD : ");
                Serial.printf("%f ", Kd);
                //Serial.println();
                break;

            case ASSERVISSEMENT_CONFIG_PERIMETRE_ROUE_CODEUSE :
                memcpy(&PERIMETRE_ROUE_CODEUSE, rxMsg[FIFO_lecture].dt, 8);
                AsserInitCoefs(Kp, Ki, Kd); 
                Serial.print("  ASSERVISSEMENT_CONFIG_PERIMETRE_ROUE_CODEUSE : ");
                Serial.printf("%f ", PERIMETRE_ROUE_CODEUSE);
                //Serial.println();
                break;
            case ASSERVISSEMENT_CONFIG_LARGEUR_ROBOT :
                memcpy(&LARGEUR_ROBOT, rxMsg[FIFO_lecture].dt, 8);
                AsserInitCoefs(Kp, Ki, Kd); 
                Serial.print("  ASSERVISSEMENT_CONFIG_LARGEUR_ROBOT : ");
                Serial.printf("%f ", LARGEUR_ROBOT);
                //Serial.println();
                break;
            case ECRAN_CHOICE_COLOR :
                //Serial.println("ECRAN_CHOICE_COLOR");
                break;
                
            case ASSERVISSEMENT_ENABLE :
                asser_actif = rxMsg[FIFO_lecture].dt[0];
                if(asser_actif == 1)
                {
                    roue_drt_init = lireCodeurD();
                    roue_gch_init = lireCodeurG();
                }
                //Serial.println("ASSERVISSEMENT_ENABLE");
            break;
                
            case ASSERVISSEMENT_DECEL :
                {
                    double k = TE/0.02;
                    VMAX = ((double)rxMsg[FIFO_lecture].dt[0]+256*rxMsg[FIFO_lecture].dt[1])*k;
                    DMAX = ((double)rxMsg[FIFO_lecture].dt[2]+256*rxMsg[FIFO_lecture].dt[3])*k*k;
                    ralentare = 1;
                    ////Serial.println("ACKNOWLEDGE_MOTEUR");
                    remplirStruct(DATArobot,ACKNOWLEDGE_MOTEUR, 2, ASSERVISSEMENT_DECEL, 0,0,0,0,0,0,0);
                    writeStructInCAN(DATArobot);
                    //CANenvoiMsg2x1Byte(ACKNOWLEDGE_MOTEUR, 0x19, 0);
                }
                ////Serial.println("ASSERVISSEMENT_DECEL");
            break;
            case ASSERVISSEMENT_XYT :
            {
                    // `#START MESSAGE_X_Y_Theta_RECEIVED` 
                    stop_receive = 0;

                    //On vide le buffer de mouvements
                    liste = (struct Ordre_deplacement){TYPE_DEPLACEMENT_IMMOBILE,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                    nb_ordres = 0;
                    cpt_ordre = 0;//ne sert à rien mais au cas où, au futur...

                    liste.x = (rxMsg[FIFO_lecture].dt[1] << 8) | rxMsg[FIFO_lecture].dt[0];
                    liste.y = (rxMsg[FIFO_lecture].dt[3] << 8) | rxMsg[FIFO_lecture].dt[2];
                    // liste.y *= -1;
                    liste.theta = (rxMsg[FIFO_lecture].dt[5] << 8) | rxMsg[FIFO_lecture].dt[4];
                    liste.sens =  rxMsg[FIFO_lecture].dt[6];
                    liste.type = TYPE_DEPLACEMENT_X_Y_THETA;
                    liste.vmax = VMAX;
                    liste.amax = AMAX;        
                    ////Serial.println("ACKNOWLEDGE_MOTEUR");
                    remplirStruct(DATArobot,ACKNOWLEDGE_MOTEUR, 2, ASSERVISSEMENT_XYT, 0,0,0,0,0,0,0);
                    writeStructInCAN(DATArobot);    
                    //CANenvoiMsg2x1Byte(ACKNOWLEDGE_MOTEUR, 0x20, 0);

                    
            }
            break;
            case ASSERVISSEMENT_COURBURE:
            {
                /* `#START MESSAGE_Rayon_de_courbure_RECEIVED` */
                stop_receive = 0;

                int16_t rayon = (rxMsg[FIFO_lecture].dt[1] << 8) | rxMsg[FIFO_lecture].dt[0];
                int16_t theta = (rxMsg[FIFO_lecture].dt[3] << 8) | rxMsg[FIFO_lecture].dt[2];
                uint8_t sens = rxMsg[FIFO_lecture].dt[4];
                uint8_t enchainement = rxMsg[FIFO_lecture].dt[5];
                uint8_t speedRatio = rxMsg[FIFO_lecture].dt[6];
        
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
                        ////Serial.println("ACKNOWLEDGE_MOTEUR");
                        remplirStruct(DATArobot,ACKNOWLEDGE_MOTEUR, 2, 0x21, 0,0,0,0,0,0,0);
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
                    ////Serial.println("ACKNOWLEDGE_MOTEUR");
                    remplirStruct(DATArobot,ACKNOWLEDGE_MOTEUR, 2, 0x21, 0,0,0,0,0,0,0);
                    writeStructInCAN(DATArobot);
                    //CANenvoiMsg2x1Byte(ACKNOWLEDGE_MOTEUR, 0x21, 0);
                }
        
            }
            break;  
            case ASSERVISSEMENT_CONFIG:
            {
                double k = TE/0.02;
                int16_t vmax = (rxMsg[FIFO_lecture].dt[1] << 8) | rxMsg[FIFO_lecture].dt[0];
                int16_t amax = (rxMsg[FIFO_lecture].dt[3] << 8) | rxMsg[FIFO_lecture].dt[2];
                VMAX = ((double)vmax)*k;
                AMAX = ((double)amax)*k*k;
                DMAX = ((double)amax)*0.75*k*k;
                ralentare = 1;
        
                //CANenvoiMsg2x1Byte(ACKNOWLEDGE_MOTEUR, 0x22, 0);
                remplirStruct(DATArobot,ACKNOWLEDGE_MOTEUR, 2, ASSERVISSEMENT_CONFIG, 0,0,0,0,0,0,0);
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

                int16_t angle = (rxMsg[FIFO_lecture].dt[1] << 8) | rxMsg[FIFO_lecture].dt[0];
        
                liste.type = TYPE_DEPLACEMENT_ROTATION;
                liste.angle = LARGEUR_ROBOT * M_PI * RESOLUTION_ROUE_CODEUSE * angle / (3600 * PERIMETRE_ROUE_CODEUSE);
                liste.vmax = VMAX;
                liste.amax = AMAX;
                ////Serial.println("ACKNOWLEDGE_MOTEUR");
                remplirStruct(DATArobot,ACKNOWLEDGE_MOTEUR, 2, ASSERVISSEMENT_ROTATION, 0,0,0,0,0,0,0);
                writeStructInCAN(DATArobot);
                //CANenvoiMsg2x1Byte(ACKNOWLEDGE_MOTEUR, 0x23, 0);
                
            }
            break;  
            case ASSERVISSEMENT_RECALAGE:
            {
                stop_receive = 0;

                int16_t distance = (rxMsg[FIFO_lecture].dt[1] << 8) | rxMsg[FIFO_lecture].dt[0];
                uint8_t mode = rxMsg[FIFO_lecture].dt[2];
                int16_t valRecalage = (rxMsg[FIFO_lecture].dt[4] << 8) | rxMsg[FIFO_lecture].dt[3];
                uint8_t enchainement = rxMsg[FIFO_lecture].dt[5];
                int8_t vinit = rxMsg[FIFO_lecture].dt[6];
                int8_t vfin = rxMsg[FIFO_lecture].dt[7];

                 
        
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

                    remplirStruct(DATArobot,ACKNOWLEDGE_MOTEUR, 2, ASSERVISSEMENT_RECALAGE, 0,0,0,0,0,0,0);
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
                        remplirStruct(DATArobot,ACKNOWLEDGE_MOTEUR, 2, ASSERVISSEMENT_RECALAGE, 0,0,0,0,0,0,0);
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

                    remplirStruct(DATArobot,ACKNOWLEDGE_MOTEUR, 2, ASSERVISSEMENT_RECALAGE, 0,0,0,0,0,0,0);
                    writeStructInCAN(DATArobot);
                    //CANenvoiMsg2x1Byte(ACKNOWLEDGE_MOTEUR, 0x24, 0);
                }
            }
            break;
            case ASSERVISSEMENT_BEZIER:
            {
                 /* `#START MESSAGE_Courbe_Bezier_RECEIVED` */
        
                // distance roue droite en ticks d'encodeur
                int32_t distRoueDroite = (rxMsg[FIFO_lecture].dt[3] << 24) | (rxMsg[FIFO_lecture].dt[2] << 16) | (rxMsg[FIFO_lecture].dt[1] << 8) | rxMsg[FIFO_lecture].dt[0];          //distance roue droite(4/4) |
                // distance roue gauche en ticks d'encodeur
                int32_t distRoueGauche = (rxMsg[FIFO_lecture].dt[7] << 24) | (rxMsg[FIFO_lecture].dt[6] << 16) | (rxMsg[FIFO_lecture].dt[5] << 8) | rxMsg[FIFO_lecture].dt[4];
        
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
                    ////Serial.println("ACKNOWLEDGE_BEZIER");
                    remplirStruct(DATArobot,ACKNOWLEDGE_BEZIER,0,0,0,0,0,0,0,0,0);
                    writeStructInCAN(DATArobot);

                    //CANenvoiMsg(ACKNOWLEDGE_BEZIER);
                }
        
        

            }
            break;  
            case ODOMETRIE_SMALL_POSITION:
            {
                Odo_x = (rxMsg[FIFO_lecture].dt[1] << 8) | rxMsg[FIFO_lecture].dt[0];
                Odo_y = (rxMsg[FIFO_lecture].dt[3] << 8) | rxMsg[FIFO_lecture].dt[2];
                Odo_theta = (rxMsg[FIFO_lecture].dt[5] << 8) | rxMsg[FIFO_lecture].dt[4];

                remplirStruct(DATArobot,ACKNOWLEDGE_MOTEUR,ODOMETRIE_SMALL_POSITION,0,0,0,0,0,0,0,0);
                writeStructInCAN(DATArobot);
            }
            break;
            case ODOMETRIE_SMALL_VITESSE:
            {

            }
            break;
            case ODOMETRIE_BIG_POSITION:
            {
                Odo_x = (rxMsg[FIFO_lecture].dt[1] << 8) | rxMsg[FIFO_lecture].dt[0];
                Odo_y = (rxMsg[FIFO_lecture].dt[3] << 8) | rxMsg[FIFO_lecture].dt[2];
                Odo_theta = (rxMsg[FIFO_lecture].dt[5] << 8) | rxMsg[FIFO_lecture].dt[4];
                remplirStruct(DATArobot,ACKNOWLEDGE_MOTEUR,ODOMETRIE_BIG_POSITION,0,0,0,0,0,0,0,0);
                writeStructInCAN(DATArobot);
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
                ////Serial.println("ACKNOWLEDGE_MOTEUR");
                remplirStruct(DATArobot,ACKNOWLEDGE_MOTEUR, 2, ASSERVISSEMENT_STOP, 0,0,0,0,0,0,0);
                writeStructInCAN(DATArobot);
                //CANenvoiMsg2x1Byte(ACKNOWLEDGE_MOTEUR, 0x01, 0);
            }
            break;  
            case CHECK_MOTEUR:
            {
                /* `#START MESSAGE_Check_RECEIVED` */
                attente = 1;
                ////Serial.println("ALIVE_MOTEUR");
                remplirStruct(DATArobot,ALIVE_MOTEUR, 0, 0, 0,0,0,0,0,0,0);
                writeStructInCAN(DATArobot);
                //CANenvoiMsg(ALIVE_MOTEUR);
            }
            break;

//---------------------------------------------Qt
            case ASSERVISSEMENT_CONFIG_KPP_Qt:{
                Kp = ((rxMsg[FIFO_lecture].dt[0] << 24) | (rxMsg[FIFO_lecture].dt[1] << 16) | 
                      (rxMsg[FIFO_lecture].dt[2] << 8) | rxMsg[FIFO_lecture].dt[3]) / 1000.000;  
                
                AsserInitCoefs(Kp, Ki, Kd);
                Serial.print("  ASSERVISSEMENT_CONFIG_KPP Qt: ");
                Serial.printf("%f ", Kp);
                //Serial.println();
                }
                break;
            case ASSERVISSEMENT_CONFIG_KPI_Qt :
                Ki = ((rxMsg[FIFO_lecture].dt[0] << 24) | (rxMsg[FIFO_lecture].dt[1] << 16) | 
                      (rxMsg[FIFO_lecture].dt[2] << 8) | rxMsg[FIFO_lecture].dt[3]) / 1000.000;  

                AsserInitCoefs(Kp, Ki, Kd);
                Serial.print("  ASSERVISSEMENT_CONFIG_KPI Qt: ");
                Serial.printf("%f ", Ki);
                //Serial.println();
                break;
            case ASSERVISSEMENT_CONFIG_KPD_Qt :
                Kd= ((rxMsg[FIFO_lecture].dt[0] << 24) | (rxMsg[FIFO_lecture].dt[1] << 16) | 
                      (rxMsg[FIFO_lecture].dt[2] << 8) | rxMsg[FIFO_lecture].dt[3]) / 1000.000;  
                     
                AsserInitCoefs(Kp, Ki, Kd); 
                Serial.print("  ASSERVISSEMENT_CONFIG_KPD Qt: ");
                Serial.printf("%f ", Kd);
                //Serial.println();
                break;
            case ASSERVISSEMENT_CONFIG_LARGEUR_ROBOT_Qt :
                LARGEUR_ROBOT = ((rxMsg[FIFO_lecture].dt[0] << 24) | (rxMsg[FIFO_lecture].dt[1] << 16) | 
                                 (rxMsg[FIFO_lecture].dt[2] << 8) | rxMsg[FIFO_lecture].dt[3]) / 100.000;  
                     
                AsserInitCoefs(Kp, Ki, Kd); 
                Serial.print("  ASSERVISSEMENT_CONFIG_LARGEUR_ROBOT_Qt: ");
                Serial.printf("%f ", LARGEUR_ROBOT);
                //Serial.println();
                break;
            case ASSERVISSEMENT_CONFIG_PERIMETRE_ROUE_CODEUSE_Qt :
                PERIMETRE_ROUE_CODEUSE= ((rxMsg[FIFO_lecture].dt[0] << 24) | (rxMsg[FIFO_lecture].dt[1] << 16) | 
                                         (rxMsg[FIFO_lecture].dt[2] << 8) | rxMsg[FIFO_lecture].dt[3]) / 100.000;  
                     
                AsserInitCoefs(Kp, Ki, Kd); 
                Serial.print("  ASSERVISSEMENT_CONFIG_PERIMETRE_ROUE_CODEUSE_Qt : ");
                Serial.printf("%f ", PERIMETRE_ROUE_CODEUSE);
                //Serial.println();
                break;
            case IDCAN_POS_XY_OBJET:{

            }break;
//---------------------------------------------
            
            default :
            break;
    }
    
    /*
    // Send message to master via bleutooth
    if (connected){
      prxRemoteCharacteristic->writeValue((uint8_t *)&rxMsg[FIFO_lecture], sizeof(rxMsg[FIFO_lecture]));
      //Serial.println("Sending via BT...");
    } else{//Serial.println("The device is not connected");}
//   }
  //if new CAN by BT are available, write it in CAN bus / CAN <-> Bt <-> CAN and use it to control the Robot
//   if (newCan){
//     newCan = false;
//     writeStructInCAN(rxMsg[FIFO_lecture]); 
//     ////Serial.println(rxMsg[FIFO_lecture].ID);
//     BtAvailable = true;
//   }*/
    FIFO_lecture=(FIFO_lecture+1)%SIZE_FIFO;
  
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

    //Calcul de la distance parcourue
    dist = 0.5*((Odo_val_pos_D - Odo_last_val_pos_D) + (Odo_val_pos_G - Odo_last_val_pos_G)); //En ticks d'encodeur

    //Calcul de la valeur de l'angle parcouru
    ang = (((Odo_val_pos_D - Odo_last_val_pos_D) - (Odo_val_pos_G - Odo_last_val_pos_G))*1800.0*PERIMETRE_ROUE_CODEUSE/(LARGEUR_ROBOT*M_PI*RESOLUTION_ROUE_CODEUSE));//En dizieme d'angle
    
    //Determination de la position sur le terrain en X, Y, Theta
    Odo_theta +=  ang;//En dizieme d'angle
    Odo_x += dist*cos((double)(Odo_theta*M_PI/1800.0))*PERIMETRE_ROUE_CODEUSE/RESOLUTION_ROUE_CODEUSE;//En mm
    Odo_y += dist*sin((double)(Odo_theta*M_PI/1800.0))*PERIMETRE_ROUE_CODEUSE/RESOLUTION_ROUE_CODEUSE;//En mm
    ////Serial.println(Odo_x); 

    //Stockage de la derniere valeur de l'odometrie
    Odo_last_val_pos_D = Odo_val_pos_D;
    Odo_last_val_pos_G = Odo_val_pos_G;

    mscount1 ++;    
    //Condition d'envoi des informations de l'odometrie par CAN 
    if(mscount1 >= (500/TE_100US))//toutes les 50ms
    {
        // digitalWrite(27, set);//pour mesurer le temps de mscount1 avec l'oscilloscope
        // set = !set;
        mscount1 = 0;
        // //Serial.println();
         //Serial.printf("Odo_val_pos_D : %lf ; Odo_val_pos_G : %lf ; Odo_val_pos_D - Odo_val_pos_G : %lf\n", Odo_val_pos_D, Odo_val_pos_G, erreur);

        CANenvoiMsg3x2Bytes(ODOMETRIE_SMALL_POSITION, Odo_x, Odo_y, ((int16_t)Odo_theta) % 3600);
        //CANenvoiMsg3x2Bytes(ODOMETRIE_SMALL_POSITION, Odo_x, Odo_y, ((int16)Odo_theta) % 3600);
        //CANenvoiMsg3x2Bytes(DEBUG_ASSERV, QuadDec_D_GetCounter(), QuadDec_G_GetCounter(), consigne_pos);
    }
}  


void test_accel(void)
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
