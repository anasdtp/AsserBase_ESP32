//----------------------------------------------------------------------Bibliotheques
#include <Arduino.h>

static char idTimer = 0; //le numéro du Timer de 0 à 3
static int prescaler = 80; // la valeur du diviseur de temps
bool flag = true; //vrai pour compter sur le front montant, faux pour compter sur le front descendant

extern volatile uint16_t mscount , mscount1 ,  ms_count2;

int totalInterrupts = 0;   // compte le nombre de declenchement de l alarme

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void onTime();//prototype de la fonction s'exécutent à chaque interruptions

void init_Timer();