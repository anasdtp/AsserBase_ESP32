/****************************************************************************************/
/*                                   CRAC_UTILITY.Cpp                                     */
/****************************************************************************************/
/****************************************************************************************/
/*                          Inclusion des bibliotheques                                 */
/****************************************************************************************/
#include "CRAC_utility.h"
#include "Arduino.h"
#include <math.h>
#include <ESP32Encoder.h>
#define VIT_MAX 1740//1740 normalement
/****************************************************************************************/
/*                          Definition des variables                                    */
/****************************************************************************************/
double KppD, KipD, KdpD, KppG, KipG, KdpG, KppR, KipR, KdpR,                             // Valeurs des correcteurs d'asservissement pour les G moteurs
       KppDa, KipDa, KdpDa, KppGa, KipGa, KdpGa,                       // Valeurs des correcteurs d'asservissement pour les 2 moteurs
        Kpp, Kip, Kdp;
double last_ErreurPosD=0, Somme_ErreurPosD=0, ErreurPosD=0;
double last_ErreurPosG=0, Somme_ErreurPosG=0, ErreurPosG=0;
double last_Delta_ErreurPos=0, somme_Delta_ErreurPos=0, Delta_ErreurPos=0;
  
const double DTIC = PERIMETRE_ROUE_CODEUSE / RESOLUTION_ROUE_CODEUSE; // longeur d'un tic de roue codeuse, en mm
const double LARGEUR_ROBOT_TIC = LARGEUR_ROBOT/(PERIMETRE_ROUE_CODEUSE / RESOLUTION_ROUE_CODEUSE); // largeur du robot en tic   //c'est pour harmoniser les unités
unsigned char tC1, tC2, tC3, tC4, tC5, nbexpr;
BUF_CIRC_DEF(buffer_distanceG, 50);
BUF_CIRC_DEF(buffer_distanceD, 50);
ESP32Encoder EncoderD;
ESP32Encoder EncoderG;
void Encodeur_Init(){
  //ESP32Encoder::useInternalWeakPullResistors=DOWN;
	// Enable the weak pull up resistors
	ESP32Encoder::useInternalWeakPullResistors=UP;
	// use pin 23 and 22 for the first encoder
	EncoderD.attachSingleEdge(39, 36);
	// use pin 39 and 36 for the second encoder
	EncoderG.attachSingleEdge(22, 23);
	// clear the encoder's raw count and set the tracked count to zero
    EncoderD.clearCount();
    EncoderG.clearCount();
}
                
/****************************************************************************************/
/* NOM : AsserInitCoefs                                                                 */
/* ARGUMENT : rien                                                                      */
/* RETOUR : rien                                                                        */
/* DESCRIPTIF : Fixe la valeur des correcteurs                                          */
/****************************************************************************************/
void AsserInitCoefs()
{
    //Coefficients de correction de l'asservissement
    Kpp = 2;//0.9     //0.4 0.5  //plus grand = roue qui forcent plus pour revenir //2 avant
    Kip = 0.001;//0.0007; le 7 juin 2021  //.0001;   //0.0007 0.0005 // suppression de Ki pour tests de reset //0 avant
    Kdp = 2;     //0.1 0.5     //plus grand = asservissement plus dur //2 avant
       
    KppD = Kpp;     //0.5
    KipD = Kip*TE/0.02;   //0.001
    KdpD = Kdp/TE*(1/320);     //2.5 /0.02
    KppG = Kpp;     //0.5
    KipG = Kip*TE/0.02;  //0.001
    KdpG = Kdp/TE*0.02;     //2.5
        
    KppR = 0;//1;     //0.5
    KipR = 0;//0.1;  //0.001
    KdpR = 0;//10;     //2.5
}
/****************************************************************************************/
/* NOM : Assert_Init_D                                                                  */
/* ARGUMENT : rien                                                                      */
/* RETOUR : rien                                                                        */
/* DESCRIPTIF : permet d'initialisation les variables globales du cote droit            */
/****************************************************************************************/
void Asser_Init_D()
{
    ErreurPosD=0;
    last_ErreurPosD=0;
    Somme_ErreurPosD=0;
}
/****************************************************************************************/
/* NOM : Assert_Init_G                                                                  */
/* ARGUMENT : rien                                                                      */
/* RETOUR : rien                                                                        */
/* DESCRIPTIF : permet d'initialisation les variables globales du cote gauche           */
/****************************************************************************************/
void Asser_Init_G()
{
    ErreurPosG=0;
    last_ErreurPosG=0;
    Somme_ErreurPosG=0;
}
/****************************************************************************************/
/* NOM : Assert_Init                                                                    */
/* ARGUMENT : rien                                                                      */
/* RETOUR : rien                                                                        */
/* DESCRIPTIF : permet d'initialisation les variables globales de chaque cote           */
/****************************************************************************************/
void Asser_Init(void)
{
    somme_Delta_ErreurPos=0;
    Asser_Init_D();
    Asser_Init_G();
}
/****************************************************************************************/
/* NOM : Asser_Pos_Mot                                                                    */
/* ARGUMENT :                                                                       */
/* RETOUR : rien                                                                        */
/* DESCRIPTIF :            */
/****************************************************************************************/
void Asser_Pos_Mot(double pcons_posG, double pcons_posD, double* commandeG, double* commandeD)
{
    static double pcons_posD_prec = 0, pcons_posG_prec = 0;
    double pcons_posG_rectif, pcons_posD_rectif; 
    double posD, Delta_ErreurPosD;
    double posG, Delta_ErreurPosG;
    double deriv_Delta_ErreurPos;
    
    
    posD = lireCodeurD();      //Recuperation de la valeur du compteur incrémental
    ErreurPosD = pcons_posD - posD;      //Calcul l'erreur par rapport à la consigne 
    
    posG = lireCodeurG();      //Recuperation de la valeur du compteur incrémental
    ErreurPosG = pcons_posG - posG;      //Calcul l'erreur par rapport à la consigne   
    
    Delta_ErreurPos = ErreurPosG - ErreurPosD;
    deriv_Delta_ErreurPos = Delta_ErreurPos - last_Delta_ErreurPos;    //Calcul de la derivee
    somme_Delta_ErreurPos += Delta_ErreurPos;     //Calcul de l'integrale   
    
    if(KipR>0)
    {
        if (somme_Delta_ErreurPos <= (-250/KipR)) 
            somme_Delta_ErreurPos = (-250/KipR);
        else if (somme_Delta_ErreurPos >= (250/KipR)) 
            somme_Delta_ErreurPos = (250/KipR);
    }
    double rectif = KppR * Delta_ErreurPos + KipR * somme_Delta_ErreurPos + KdpR * deriv_Delta_ErreurPos;
    pcons_posG_rectif = pcons_posG + rectif;        //Mise a jour de la commande moteur
    pcons_posD_rectif = pcons_posD - rectif;        //Mise a jour de la commande moteur
    last_Delta_ErreurPos = Delta_ErreurPos;       //Stockage de la nouvelle erreur
    
    //*commandeG = Asser_Pos_MotG(pcons_posG_rectif);
    //*commandeD = Asser_Pos_MotD(pcons_posD_rectif);
    
    
    posD = lireCodeurD();      //Recuperation de la valeur du compteur incrémental
    ErreurPosD = pcons_posD_rectif - posD;      //Calcul l'erreur par rapport à la consigne   
    Delta_ErreurPosD = ErreurPosD - last_ErreurPosD;    //Calcul de la derivee
    Somme_ErreurPosD += ErreurPosD;     //Calcul de l'integrale 
    
    posG = lireCodeurG();      //Recuperation de la valeur du compteur incrémental
    ErreurPosG = pcons_posG_rectif - posG;      //Calcul l'erreur par rapport à la consigne   
    Delta_ErreurPosG = ErreurPosG - last_ErreurPosG;    //Calcul de la derivee
    Somme_ErreurPosG += ErreurPosG;     //Calcul de l'integrale      
    
    if(KipD>0)
    {
        if (Somme_ErreurPosD <= (-250/KipD)) 
            Somme_ErreurPosD = (-250/KipD);
        else if (Somme_ErreurPosD >= (250/KipD)) 
            Somme_ErreurPosD = (250/KipD);
    }
    *commandeD = (KppD * ErreurPosD + KipD * Somme_ErreurPosD + KdpD * Delta_ErreurPosD);        //Mise a jour de la commande moteur
    last_ErreurPosD = ErreurPosD;       //Stockage de la nouvelle erreur
    if(KipG>0)
    {
        if (Somme_ErreurPosG <= (-250/KipG)) 
            Somme_ErreurPosG = (-250/KipG);
        else if (Somme_ErreurPosG >= (250/KipG)) 
            Somme_ErreurPosG = (250/KipG);
    }
    *commandeG = (KppG * ErreurPosG + KipG * Somme_ErreurPosG + KdpG * Delta_ErreurPosG);        //Mise a jour de la commande moteur
    last_ErreurPosG = ErreurPosG;       //Stockage de la nouvelle erreur
    
    
    //pcons_posD_prec = pcons_posD, pcons_posG_prec = pcons_posG;
}
/******************************/
/* NOM : Asser_Pos_MotD                                                               */
/* ARGUMENT : double pcons_pos -> position voulue exprimée en tick d'encodeur           */
/*                                1024 par tour de roue                                 */
/* RETOUR : rien                                                                        */
/* DESCRIPTIF : Fonction appelee pour calculer la commande du moteur D                  */
/******************************/ 
double Asser_Pos_MotD(double pcons_pos)
{
    double pos, Delta_ErreurPos, commande;
    pos = lireCodeurD();      //Recuperation de la valeur du compteur incrémental
    ErreurPosD = pcons_pos - pos;      //Calcul l'erreur par rapport à la consigne   
    Delta_ErreurPos = ErreurPosD - last_ErreurPosD;    //Calcul de la derivee
    Somme_ErreurPosD += ErreurPosD;     //Calcul de l'integrale    
    if(KipD>0)
    {
        if (Somme_ErreurPosD <= (-250/KipD)) 
            Somme_ErreurPosD = (-250/KipD);
        else if (Somme_ErreurPosD >= (250/KipD)) 
            Somme_ErreurPosD = (250/KipD);
    }
    commande = (KppD * ErreurPosD + KipD * Somme_ErreurPosD + KdpD * Delta_ErreurPos);        //Mise a jour de la commande moteur
    last_ErreurPosD = ErreurPosD;       //Stockage de la nouvelle erreur
    return commande;
}
/******************************/
/* NOM : Asser_Pos_MotG                                                                */
/* ARGUMENT : double pcons_pos -> position voulue exprimée en tick d'encodeur           */
/*                                1024 par tour de roue                                 */
/* RETOUR : rien                                                                        */
/* DESCRIPTIF : Fonction appelee pour calculer la commande du moteur D                  */
/******************************/
double Asser_Pos_MotG(double pcons_pos)
{
    double pos, Delta_ErreurPos, commande;
    pos = lireCodeurG();      //Recuperation de la valeur du compteur incrémental
    ErreurPosG = pcons_pos - pos;      //Calcul l'erreur par rapport à la consigne   
    Delta_ErreurPos = ErreurPosG - last_ErreurPosG;    //Calcul de la derivee
    Somme_ErreurPosG += ErreurPosG;     //Calcul de l'integrale    
    if(KipG>0)
    {
        if (Somme_ErreurPosG <= (-250/KipG)) 
            Somme_ErreurPosG = (-250/KipG);
        else if (Somme_ErreurPosG >= (250/KipG)) 
            Somme_ErreurPosG = (250/KipG);
    }
    commande = (KppG * ErreurPosG + KipG * Somme_ErreurPosG + KdpG * Delta_ErreurPos);        //Mise a jour de la commande moteur
    last_ErreurPosG = ErreurPosG;       //Stockage de la nouvelle erreur
    return commande;
}
/****************************************************************************************/
/* NOM : lireCodeurD                                                                    */
/* ARGUMENT : rien                                                                      */
/* RETOUR : double                                                                      */
/* DESCRIPTIF : permet de lire la valeur de l'encodeur droit                            */
/****************************************************************************************/
double lireCodeurD(void)
{
    return COEF_ROUE_DROITE*EncoderD.getCount();
}
/****************************************************************************************/
/* NOM : lectureErreur                                                                  */
/* ARGUMENT : long pcons : consigne fixée                                               */
/* RETOUR : char                                                                        */
/* DESCRIPTIF : permet de lire la valeur de l'encodeur droit                            */
/****************************************************************************************/
void lectureErreur(void)
{
    if ((ErreurPosD >= EXPLOSION_TAUX) || (ErreurPosG >= EXPLOSION_TAUX)){
        liste.type = TYPE_MOUVEMENT_SUIVANT;
        Message_Fin_Mouvement = ASSERVISSEMENT_ERREUR;
    }
}
/****************************************************************************************/
/* NOM : lireCodeurG                                                                    */
/* ARGUMENT : rien                                                                      */
/* RETOUR : double                                                                      */
/* DESCRIPTIF : permet de lire la valeur de l'encodeur gauche                           */
/****************************************************************************************/
double lireCodeurG(void)
{
    return COEF_ROUE_GAUCHE*EncoderG.getCount();
}
/****************************************************************************************/
/* NOM : write_PWMD                                                                     */
/* ARGUMENT : entier                                                                    */
/* RETOUR : rien                                                                        */
/* DESCRIPTIF : permet de choisir la vitesse du moteur 1                                */
/****************************************************************************************/
void write_PWMD(int vit)
{
    if(vit >= 0)                    //Mode Avancer 
    {
        Moteur_D_INA_Write(1);      
        Moteur_D_INB_Write(0);
        if(vit > VIT_MAX) vit = VIT_MAX;    //Palier de vitesse fixé à 250
        PWM_D_WriteCompare(vit);    
    }
    else                            //Mode Reculer
    {
        Moteur_D_INA_Write(0);
        Moteur_D_INB_Write(1);
        if(vit < -VIT_MAX) vit = -VIT_MAX;  //Palier de vitesse fixé à 250
        PWM_D_WriteCompare(-vit);
        
    }
/*    if(stop_receive)
    {
        Moteur_D_INA_Write(1);      
        Moteur_D_INB_Write(1);
        PWM_D_WriteCompare(vit);
        Asser_Init();
    }*/    
}
/****************************************************************************************/
/* NOM : write_PWMG                                                                     */
/* ARGUMENT : entier                                                                    */
/* RETOUR : rien                                                                        */
/* DESCRIPTIF : permet de choisir la vitesse du moteur 2                                */
/****************************************************************************************/
void write_PWMG(int vit)
{
    if(vit >= 0)                    //Mode Avancer
    {
    
        Moteur_G_INA_Write(0);
        Moteur_G_INB_Write(1);
        if(vit > VIT_MAX) vit = VIT_MAX;    //Palier de vitesse fixé à 250
        PWM_G_WriteCompare(vit);
    }
    else                            //Mode Reculer
    {
        
        Moteur_G_INA_Write(1);
        Moteur_G_INB_Write(0);
        if(vit < -VIT_MAX) vit = -VIT_MAX;  //Palier de vitesse fixé à 250
        PWM_G_WriteCompare(-vit);
    }
/*    if(stop_receive)
    {
        Moteur_G_INA_Write(1);      
        Moteur_G_INB_Write(1);
        PWM_G_WriteCompare(vit);
        Asser_Init();
    }*/    
}
/***************************************************************************************
 NOM : Arret                                                                          
 ARGUMENT : void                                                                      
 RETOUR : rien                                                                        
 DESCRIPTIF : Fonction appelee pour effectuer un arret                                
***************************************************************************************/
void Arret_Brutal(void)
{
        Moteur_D_INA_Write(1);     
        Moteur_D_INB_Write(1);
        Moteur_G_INA_Write(1);      
        Moteur_G_INB_Write(1);
        PWM_G_WriteCompare(0);
        PWM_D_WriteCompare(0);
        Asser_Init();
}
void Arret(void)
{    
    
    //Ecrire un PWM egal a 0
    write_PWMG(0);   
    write_PWMD(0);
}
int signesDif(double v1, double v2)
{
    return (v1>=0) != (v2>=0);
}
void Moteur_G_INA_Write(bool set){
  digitalWrite(26, set);
}
void Moteur_G_INB_Write(bool set){
  digitalWrite(25, set);
}
void Moteur_D_INA_Write(bool set){
  digitalWrite(16, set);
}
void Moteur_D_INB_Write(bool set){
  digitalWrite(15, set);
}
void PWM_D_WriteCompare(int vit){
  ledcWrite(1, vit*1000/VIT_MAX);
}
void PWM_G_WriteCompare(int vit){
  ledcWrite(0, vit*1000/VIT_MAX);
}
/* [] END OF FILE */