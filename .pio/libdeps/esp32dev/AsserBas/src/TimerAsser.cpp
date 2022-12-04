/****************************************************************************************/
/*                          Inclusion des bibliotheques                                 */
/****************************************************************************************/
#include "TimerAsser.h"


static char idTimer = 0; //le numéro du Timer de 0 à 3
static int prescaler = 80; // la valeur du diviseur de temps
bool flag = true; //vrai pour compter sur le front montant, faux pour compter sur le front descendant





hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void onTime() {//fonction s'exécutent à chaque interruptions 
   mscount++;
}

void init_Timer(){
    // Configure le Prescaler a 80 le quartz de l ESP32 est cadence a 80Mhz => à vérifier pour l'esp32-32E, peut etre 40Mhz?
   // 80000000 / 80 = 1000000 tics / seconde
   timer = timerBegin(idTimer, prescaler, flag);                
   timerAttachInterrupt(timer, &onTime, true);//fait qu'on execute la fonction onTime à chaque interruptions
    
   // Regle le declenchement d une alarme chaque seconde
   timerAlarmWrite(timer, 4, true);      //freq de 250 000 Hz    
   timerAlarmEnable(timer); //active l'alarme
}
/* [] END OF FILE */
