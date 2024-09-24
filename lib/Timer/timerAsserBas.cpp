#include "timerAsserBas.h"
#include "CRAC_utility.h"
#include <espCan.h>

hw_timer_t * timer = NULL;

void onTime() {//fonction s'exécutent à chaque interruptions 
   mscount++;
}
//Timer
void init_Timer(){
  static char idTimer = 0; //le numéro du Timer de 0 à 3
  static int prescaler = 8000; // la valeur du diviseur de temps
  bool flag = true; //vrai pour compter sur le front montant, faux pour compter sur le front descendant
    // Configure le Prescaler a 80 le quartz de l ESP32 est cadence a 80Mhz => à vérifier pour l'esp32-32E, peut etre 40Mhz?
   // 80000000 / 80 = 1000000 tics / seconde
   timer = timerBegin(idTimer, prescaler, flag);
   timerAttachInterrupt(timer, &onTime, true);//fait qu'on execute la fonction onTime à chaque interruptions
    
   // Regle le declenchement d une alarme chaque seconde
   timerAlarmWrite(timer, 1, true);      //freq de 250 000 Hz    
   timerAlarmEnable(timer); //active l'alarme
   //Serial.println("Fin init Timer");
}

bool TempsEchantionnage(int TIME){
    if (mscount >= TIME) 
  {   
    //Serial.println("erreur temp calcul");
    //Serial.println(mscount);
    remplirStruct(DATArobot,ERREUR_TEMP_CALCUL,2, mscount,TE_100US,0,0,0,0,0,0);
    writeStructInCAN(DATArobot);
  }
  else 
  {
    while (mscount<(TIME));
  }
  mscount = 0; 
  //digitalWrite(27, set);//pour mesurer le temps de boucle avec l'oscilloscope
  //set = !set; temps de boucle = 1/(freq/2)
  return true;
}

bool TempsEchantionnageSansBlocage(uint16_t TIMEMS){ // ms
    static uint32_t LastMscount = millis();
    if ((millis() - LastMscount) >= TIMEMS)
    {
        LastMscount = millis();
        return true;
    }
    else
    {
        return false;
    }
}