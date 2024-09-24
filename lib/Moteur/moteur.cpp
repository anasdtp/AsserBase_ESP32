#include "moteur.h"
#include <CRAC_utility.h>

void Moteur_Init(int inApinMOTD, int inApinMOTG, int inBpinMOTD, int inBpinMOTG, int PWMMOTD, int PWMMOTG, int PWMDChannel, int PWMGChannel){
  //init pins carte de puiss
  pinMode(inApinMOTD, OUTPUT);
  pinMode(inApinMOTD, OUTPUT);
  pinMode(inBpinMOTG, OUTPUT);
  pinMode(inBpinMOTD, OUTPUT);
  pinMode(PWMMOTG, OUTPUT);
  pinMode(PWMMOTD, OUTPUT);
  pinMode(27, OUTPUT);//Je sais plus à quoi correspond cette pin //Okay?? :')
  //mise à l'arret
  digitalWrite(inApinMOTG, LOW);
  digitalWrite(inApinMOTD, LOW);
  digitalWrite(inBpinMOTG, LOW);
  digitalWrite(inBpinMOTD, LOW);
  //init pwm
  setupPWM(PWMMOTD, PWMDChannel);
  setupPWM(PWMMOTG, PWMGChannel);
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
  const int freqMot = 20000, resolution = 11;
  ledcSetup(PWMChannel, freqMot, resolution);
  ledcAttachPin(PWMpin, PWMChannel);
}
