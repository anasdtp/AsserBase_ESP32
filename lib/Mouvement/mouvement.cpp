#include "mouvement.h"
#include <espCan.h>
#include <CRAC_utility.h>
#include <ident_crac.h>
#include <buffer_circulaire.h>
#include <clotho.h>
#include "math.h"
double          consigne_pos, consigne_vit,                 // Consignes de position et de vitesse dans les mouvements
                VMAX, AMAX, AMAX_CLO, DMAX,                 // Valeurs maximales d'accéleration, décélération et vitesse
                Odo_x, Odo_y, Odo_theta, Odo_val_pos_D, Odo_val_pos_G, Odo_last_val_pos_D, Odo_last_val_pos_G,  // Variables de positions utilisées pour l'odométrie
                roue_drt_init, roue_gch_init,               // Valeur des compteurs (!= 0) quand on commence un nouveau mouvement
                global_ta_stop=0,global_decel_stop=0,
                old_posD=0,old_posG=0;

/***************************************************************************************
 NOM : Mouvement Elementaire                                                           
 ARGUMENT : long pcons -> distance a parcourir (>0 : avancer et <0 : reculer          
            short vmax -> vitesse maximale                                            
            short amax -> acceleration maximale                                       
            short dmax -> deceleration maximale                                      
            char mvt -> choix entre rotation et ligne droite                          
 RETOUR : rien                                                                        
 DESCRIPTIF : Fonction appelee pour effectuer une ligne droite       

 Cette fonction nommée "Mouvement_Elementaire" prend en entrée un long, pcons, un char, mvt, et trois shorts, vmax, amax et dmax. Elle n'a pas de sortie. Voici une description de la fonction par section:

Déclaration des variables: Les variables déclarées sont:
tc: temps de déplacement à vitesse constante
ta: temps d'accelération
td: temps de décélération
vmax_tri: vitesse maximale atteinte dans le cas d'un profil en triangle
accel: accélération (valeur du paramètre amax divisé par 1000)
decel: décélération (valeur du paramètre dmax divisé par 1000)
pos_triangle: position dans le profil de mouvement où la vitesse maximale est atteinte
memo_etat_automate: état précédent de l'automate (initialisé à 0)
defaut_asserv: défaut d'asservissement (initialisé à 0)
Switch statement pour l'automate de gestion du déplacement:

Case INITIALISATION: 
remise à zéro des variables de consigne de position et de vitesse, du compteur, et de defaut_asserv. 
Détermination du temps d'accelération (ta) et de décélération (td) en fonction de vmax, amax et dmax. 
Calcul de la position (pos_triangle) dans le profil de mouvement où la vitesse maximale est atteinte. 
Si cette position est inférieure en valeur absolue à pcons, on utilise un profil de trapèze. 
Sinon, on utilise un profil de triangle. Envoi d'un message CAN contenant la consigne de position et 
le nombre de pas de codeurs.
Case ACCELERATION_TRAPEZE: incrémentation de la consigne de vitesse par la valeur de l'acceleration, 
puis incrémentation de la consigne de position. Si le compteur atteint ta, l'automate passe à l'état 
VITESSE_CONSTANTE_TRAPEZE, le compteur est remis à zéro, et la consigne de vitesse est mise à vmax.
Case VITESSE_CONSTANTE_TRAPEZE: incrémentation de la consigne de position. Si le compteur atteint tc 
(temps de déplacement à vitesse constante), l'automate passe à l'état DECELERATION_TRAPEZE, le compteur 
est remis à zéro.
Case DECELERATION_TRAPEZE: décrémentation de la consigne de vitesse par la valeur de la décélération, 
puis incrémentation de la consigne de position. Si le compteur atteint td (temps de décélération), 
l'automate passe à l'état ARRET, le compteur est remis à zéro, et le défaut d'asservissement est mis à 0.
Les commentaires dans le code indiquent que la fonction était utilisée pour piloter un robot sur une ligne. 
On y trouve des appels à des fonctions pour envoyer des messages CAN qui doivent communiquer avec d'autres 
éléments du système.                 (ChatGPT encore)
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
        ////Serial.println("ID_DIST_TIC_GENE");
        remplirStruct(DATArobot,ID_DIST_TIC_GENE, 2, (pcons&0xFF), ((pcons&0xFF00)<<8),0,0,0,0,0,0);
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
