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


              Cette fonction "Rayon_De_Courbure" permet de faire avancer le robot en suivant un arc de cercle. 
              Elle prend en entrée le rayon de l'arc de cercle à parcourir, l'angle à parcourir (en dixièmes de degré), 
              la vitesse maximale, la décélération maximale, l'accélération maximale et le sens de déplacement. 
              La fonction ne renvoie rien.
              Le principe est de faire avancer le robot en suivant un arc de cercle situé à une distance rayon du centre du robot. 
              Pour cela, la roue intérieure dans le cercle va moins avancer que l'autre roue. Le rapport de distance entre les 2 roues 
              est calculé par : distance de la roue intérieure par rapport au centre du cercle, divisé par distance de la roue 
              extérieure par rapport au centre.
              La fonction utilise un automate pour gérer le déplacement. Elle commence par l'état d'initialisation et de calcul
               des variables, où elle remet à zéro les variables de consigne de position et de vitesse et calcule la position voulue. 
               Elle calcule également le rapport pour la roue qui parcourt la plus petite distance et le profil de vitesse 
               (taux d'accélération, temps d'accélération et de décélération). Si la distance à parcourir est inférieure à une certaine 
               valeur, la fonction utilise un profil de trapèze. Sinon, elle utilise un profil de triangle.
               Ensuite, la fonction passe à l'état d'acceleration en profil trapeze, où elle incrémente la consigne de vitesse par 
               la valeur de l'acceleration et la consigne de position. Lorsque le temps d'acceleration est atteint, la fonction passe 
               à l'état de vitesse constante en profil trapeze, où la consigne de vitesse est constante. Lorsque le robot doit commencer 
               à décélérer, la fonction passe à l'état de deceleration en profil trapeze, où elle décélère jusqu'à s'arrêter. 
               Lorsque le robot est à l'arrêt, la fonction est terminée.  (Merci)
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
    static double dist = 0, ang1 = 0, ang2 = 0;
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
                    Serial.printf("X_Y_Theta : Dist : %f, ang1 : %hd, ang2 : %hd       ;       ", dist, ang1, ang2); 
                    // On transforme les résultats en distance et angles utilisables avec les fonctions déjà définies
                    dist = dist * (RESOLUTION_ROUE_CODEUSE / PERIMETRE_ROUE_CODEUSE);
                    ang1 = LARGEUR_ROBOT * M_PI * RESOLUTION_ROUE_CODEUSE * ang1 / (3600 * PERIMETRE_ROUE_CODEUSE);
                    ang2 = LARGEUR_ROBOT * M_PI * RESOLUTION_ROUE_CODEUSE * ang2 / (3600 * PERIMETRE_ROUE_CODEUSE);

                    Serial.printf("X_Y_Theta : Dist : %f, ang1 : %hd, ang2 : %hd\n", dist, ang1, ang2); 
                    
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
                ////Serial.println("INSTRUCTION_END_MOTEUR");
                remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 2, 0x30, 0x00,0,0,0,0,0,0);
                writeStructInCAN(DATArobot);
                //CANenvoiMsg2x1Byte(INSTRUCTION_END_MOTEUR, 0x30, 0);
                
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
                ////Serial.println("INSTRUCTION_END_MOTEUR");
                remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 2, 0x40, 0x00,0,0,0,0,0,0);
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


void init_coef(){
    double k = TE/0.02;
    VMAX = Vmax_coef*k;       // 50       petit 600   gros 600
    AMAX = Amax_coef*k*k;    //  500     petit 6000  gros 6000
    AMAX_CLO = Ama_clo_coef*k*k;    //  500     petit 6000  gros 6000
    DMAX = Dmax_coef*0.75*k*k;   //  500    petit 6000   gros 6000
    liste = (struct Ordre_deplacement){TYPE_DEPLACEMENT_IMMOBILE,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
}








clothoStruc maClotho;
double consigne_posG = 0, consigne_posD = 0;
float distanceG = 0, distanceD = 0;
char flagDebutBezier = 0;

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
            ////Serial.println("vfinAbs =0! Warning!");
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



/**
 * 
 * 
 * 1 Mesurez la distance entre le robot et l'objet, et utilisez cette distance pour déterminer le rayon de courbure de la clothoïde. 
 * 2 Plus l'objet est proche, plus la courbure de la clothoïde doit être grande.
 * 3 Déterminez la position cible que le robot doit atteindre pour éviter l'obstacle. 
 * 4 Cette position sera située sur la trajectoire en clothoïde.
 * 5 Planifiez la trajectoire en clothoïde en utilisant le rayon de courbure et la position cible.
 * 6 Faites avancer le robot en suivant la trajectoire en clothoïde.
 * 7 Répétez les étapes 1 à 4 jusqu'à ce que le robot atteigne sa destination finale.**/
void trait_Rayon_De_Courbure_Clotho(struct Ordre_deplacement* monDpl)
{
    unsigned long tc, ta, td;   //tc temps à vitesse constante, ta en acceleration, td en deceleration, temp en nombre d'étt d'automate
    double accel = 0;
    int pcons, rapport;
    
    #if F_DBUG_TRAIT_ETAT_CLOTHO
    ////Serial.println("ID_TRAIT_CLOTHO");
    remplirStruct(DATArobot,ID_TRAIT_CLOTHO, 1, 0x01, 0,0,0,0,0,0,0);
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
    remplirStruct(DATArobot,ID_CLOTHO_IMPOSSIBLE, 1, flagVir, 0,0,0,0,0,0,0);
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
    remplirStruct(DATArobot,ID_TRAIT_CLOTHO, 1, 2, 0,0,0,0,0,0,0);
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
                ////Serial.println("0x002");
                remplirStruct(DATArobot,0x002,0,0,0,0,0,0,0,0,0);
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
