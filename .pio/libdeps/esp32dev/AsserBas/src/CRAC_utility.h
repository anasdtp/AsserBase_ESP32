/****************************************************************************************/
/*                                   CRAC_UTILITY.H                                     */
/****************************************************************************************/
/****************************************************************************************/
/*                          Securite contre les multi-inclusions                        */
/****************************************************************************************/
#ifndef CRAC_UTILITYH
#define CRAC_UTILITYH
/****************************************************************************************/
/*                          Inclusion des bibliotheques                                 */
/****************************************************************************************/   
#include <Arduino.h>
#include "ident_crac.h"
#include "buffer_circulaire.h"
/****************************************************************************************/
/*                          Constantes de preprocesseur                                 */
/****************************************************************************************/    
#define F_DBUG_GENERAL 1    
    
#if F_DBUG_GENERAL
    #define DBUG 0
    #define F_DEBUG_ASSERV 0
    
    #define F_DBUG_TRAIT_ETAT 1
    #define F_DBUG_TRAIT_ETAT_CLOTHO 1
    #define F_DBUG_TRAIT_ETAT_GENE 0
    
    #define F_DBUG_LIGNE 0
    #define F_DBUG_LIGNE_GEN 0
    #define F_DBUG_LIGNE_GEN_VIT 0
    #define F_DBUG_LIGNE_GEN_TPS 0
    
    #define F_DBUG_CLOTHO_VIT 0
    #define F_DBUG_CLOTHO_TPS 0  
    #define F_DBUG_CLOTHO_DOUBLE 0 
    
    #define F_DBUG_CLOTHO_TRAIT_VIT 0
    #define F_DBUG_CLOTHO_TRAIT_TPS 0  
    #define F_DBUG_CLOTHO_TRAIT_DOUBLE 0 
    
    #define F_DBUG_TEMPS_CALCUL_CLOTHO 0
    
    #define F_DBUG_ETAT 1
    #define F_DBUG_ETAT_DPL 1  
    #define F_DBUG_ETAT_MS 1 
    
#endif    
    
#define MOTDPIN 
#define FACTEUR_DIVISION    1
#define RESOLUTION_ROUE_CODEUSE 2048        
#define RECALAGE_TH 150     //150    -- 1000    -- 512
#define EXPLOSION_TAUX 2500
    
#define MOUVEMENT_LIGNE_DROITE 0    
#define MOUVEMENT_ROTATION 1
    
#define TYPE_END_GAME 0            
#define TYPE_DEPLACEMENT_IMMOBILE 1             
#define TYPE_DEPLACEMENT_LIGNE_DROITE 2
#define TYPE_DEPLACEMENT_ROTATION 3
#define TYPE_DEPLACEMENT_X_Y_THETA 4
#define TYPE_DEPLACEMENT_RAYON_COURBURE 5
#define TYPE_DEPLACEMENT_RECALAGE 6
#define TYPE_ASSERVISSEMENT_DESACTIVE 7
#define TYPE_MOUVEMENT_SUIVANT 8
#define TYPE_INIT_STOP 9
#define TYPE_STOP 10
#define TYPE_DEPLACEMENT_RAYON_COURBURE_CLOTHOIDE 11
#define TYPE_DEPLACEMENT_LIGNE_DROITE_EN 12
#define TYPE_TRAIT_ENCH_RCV 13
#define TYPE_DEPLACEMENT_BEZIER 14
 
#define INITIALISATION 0
#define ACCELERATION_TRAPEZE 1
#define VITESSE_CONSTANTE_TRAPEZE 2
#define DECELERATION_TRAPEZE 3
#define ARRET 4
#define ACCELERATION_TRIANGLE 5
#define DECELERATION_TRIANGLE 6
#define STOP 7
#define TROP_D_ERREUR_ASSERV 8
#define ARRET_STOP 9
#define INIT_RECALAGE 0
#define ACCELERATION_RECALAGE 1
#define VITESSE_CONSTANTE_RECALAGE 2
#define FIN_RECALAGE 3
#define INIT_X_Y_THETA 0
#define ROTATION_X_Y_THETA_1 1
#define LIGNE_DROITE_X_Y_THETA 2
#define ROTATION_X_Y_THETA_2 3

#define VIT_MAX 1000//1740 normalement
/****************************************************************************************/
/*                         Definition des informations du robot                         */
/****************************************************************************************/ 
#define LARGEUR_ROBOT 214.4  //244.5 213.9 209. 208.53 ancienne valeur 213.9 valeur 10 tours 260.17  //premier robot 213.9 
#define PERIMETRE_ROUE_CODEUSE 162.9 //156.9 robot 1 // 160.9 robot 2      //theorique : 157
#define COEF_ROUE_GAUCHE 1.0   //1.00595 - 0.9992505621    Petit virage à droite à gauche  ancien : 1.0100 / 1.0055
#define COEF_ROUE_DROITE 1.0  //BLANC
#define TE_100US 25          //Temps d'echantiollonage -> 25 x 100US = 2.5ms
#define Vmax_coef 300.0           //  600       DANGER>>>>>>> chauffage micro
#define Amax_coef 3000.0          // 6000       DANGER>>>>>>> chauffage micro
#define Dmax_coef 3000.0          // 6000       DANGER>>>>>>> chauffage micro
#define Ama_clo_coef 1500.0
#define TE (TE_100US*0.0001) //soit 2.5ms
void Encodeur_Init();

void write_PWMD(double vit);
void write_PWMG(double vit);
void Moteur_G_INA_Write(bool set);
void Moteur_G_INB_Write(bool set);
void Moteur_D_INA_Write(bool set);
void Moteur_D_INB_Write(bool set);
void PWM_D_WriteCompare(int vit);
void PWM_G_WriteCompare(int vit);
void Arret(void);
void Arret_Brutal(void);
void Asser_Pos_Mot(double pcons_posG, double pcons_posD, double* commandeG, double* commandeD);
void test_accel(void);
/****************************************************************************************/
/* NOM : Asser_Pos_MotD                                                               */
/* ARGUMENT : double pcons_pos -> position voulue exprimée en tick d'encodeur           */
/*                                1024 par tour de roue                                 */
/* RETOUR : rien                                                                        */
/* DESCRIPTIF : Fonction appelee pour calculer la commande du moteur D                  */
/****************************************************************************************/ 
double Asser_Pos_MotD(double pcons_pos);
/****************************************************************************************/
/* NOM : Asser_Pos_MotG                                                                */
/* ARGUMENT : double pcons_pos -> position voulue exprimée en tick d'encodeur           */
/*                                1024 par tour de roue                                 */
/* RETOUR : rien                                                                        */
/* DESCRIPTIF : Fonction appelee pour calculer la commande du moteur G                  */
/****************************************************************************************/
double Asser_Pos_MotG(double pcons_pos);
double lireCodeurD();
double lireCodeurG();
void Asser_Init_D();
void Asser_Init_G();
void Asser_Init();
void AsserInitCoefs();
int signesDif(double v1, double v2);
/****************************************************************************************/
/* NOM : lectureErreur                                                                  */
/* ARGUMENT : void                                                                      */
/* RETOUR : void                                                                        */
/* DESCRIPTIF : permet de lire la valeur de l'encodeur droit                            */
/****************************************************************************************/
void lectureErreur(void);
/****************************************************************************************/
/*                          Definition de la structure deplacement                      */
/****************************************************************************************/
struct Ordre_deplacement
{
    char type, enchainement;
    double vmax, amax, vinit, vfin, dmax;
    short distance, recalage, val_recalage;
    long angle;
    short x, y, theta;
    signed char sens;
    short rayon, vit_ray, theta_ray;
    double ta, tc, td; //temps d'acceleration, constant et de deceleration
};
                
                
/****************************************************************************************/
/*                            Declaration des variables                                 */
/****************************************************************************************/ 
extern double   consigne_pos, consigne_vit,                 // Consignes de position et de vitesse dans les mouvements
                ErreurPosD, ErreurPosG,
                VMAX, AMAX, AMAX_CLO, DMAX,                                               // Valeurs maximales d'accéleration, décélération et vitesse
                Odo_x, Odo_y, Odo_theta, Odo_val_pos_1, Odo_val_pos_2, Odo_last_val_pos_1, Odo_last_val_pos_2,  // Variables de positions utilisées pour l'odométrie
                roue_drt_init, roue_gch_init;                                   // Valeur des compteurs (!= 0) quand on commence un nouveau mouvement
extern short    etat_automate, etat_automate_depl, new_message,
                xytheta_sens, next_move_xyt, next_move, i, stop, stop_receive, param_xytheta[3],
                etat_automate_xytheta, ralentare;
              
extern unsigned short cpt;
                
extern int cpt_ordre;
                
extern char     vitesse_danger, Stop_Danger, asser_actif, attente, Fin_Match, Message_Fin_Mouvement;
extern int      nb_ordres;
                
extern struct Ordre_deplacement liste;
extern double   KppD, KipD, KdpD, KppG, KipG, KdpG,                             // Valeurs des correcteurs d'asservissement pour les G moteurs
                KppDa, KipDa, KdpDa, KppGa, KipGa, KdpGa;                       // Valeurs des correcteurs d'asservissement pour les 2 moteurs
extern const double DTIC;   
extern const double LARGEUR_ROBOT_TIC;                     // Valeurs des correcteurs d'asservissement pour les 2 moteurs
extern unsigned char tC1, tC2, tC3, tC4, tC5, nbexpr;
extern volatile uint16_t mscount , mscount1 ,  mscount2;
    
extern buf_circ_t buffer_distanceG;
extern buf_circ_t buffer_distanceD;
extern char flagDebutBezier;

const int PWM_MOTD = 18;    // PIN18
const int PWM_MOTG = 17;    
const int PWMDChannel = 3;
const int PWMGChannel = 2;
#endif
/****************************************************************************************/
//Sinon, détails : les angles sont exprimés en dixièmes de degrés quand il faut faire des calculs, ou quand ils faut les transmettre en CAN
//les distances, pareil, c'est de millimètres
//Par contre, pour les fonctions Ligne_droite, Rotation, et asser_pos c'est exprimé en ticks d'encodeur
/****************************************************************************************/
/* [] END OF FILE */