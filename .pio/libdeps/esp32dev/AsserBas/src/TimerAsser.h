//----------------------------------------------------------------------Bibliotheques
#include <Arduino.h>

extern volatile uint16_t mscount , mscount1 ,  ms_count2;

void onTime();//prototype de la fonction s'exécutent à chaque interruptions

void init_Timer();