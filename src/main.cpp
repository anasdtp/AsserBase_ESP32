//----------------------------------------------------------------------Bibliotheques
#include <Arduino.h>

#include "espBLEcan.h"
#include <ESP32Encoder.h>
#include "CRAC_utility.h"
#include "ident_crac.h"
//----------------------------------------------------------------------Variables

CANMessage myData;//data received by BT
CANMessage DATAtoSend;//to send on BT

CANMessage DATAtoControl;//data to control the robot
CANMessage DATArobot;//DATA that the robot will send


double Odo_x, Odo_y, Odo_theta, Odo_val_pos_D, Odo_val_pos_G, Odo_last_val_pos_D, Odo_last_val_pos_G;  // Variables de positions utilisées pour l'odométrie

int inApin_MOTD = 16; // INA2 checked
int inApin_MOTG = 26; // INA1 checked
int inBpin_MOTD = 15; // INB2 checked
int inBpin_MOTG = 25; // INB1 checked
  // PWM moteur
int PWM_MOTD = 18;    // PIN18
int PWM_MOTG = 17;    

int PWMDChannel = 0;
int PWMGChannel = 1;

int joyMot = 500;
double erreur = 0;

static char idTimer = 0; //le numéro du Timer de 0 à 3
static int prescaler = 80; // la valeur du diviseur de temps
bool flag = true; //vrai pour compter sur le front montant, faux pour compter sur le front descendant
int totalInterrupts = 0;   // compte le nombre de declenchement de l alarme

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint16_t mscount , mscount1 ,  ms_count2;
void onTime();//prototype de la fonction s'exécutent à chaque interruptions
void init_Timer();

//varibles declarées dans CRAC_utility.h
void calcul(void);
void Odometrie(void);
void asser_position();
//----------------------------------------------------------------------prototypes fonctions BLE et CAN
void setupCAN();
void writeStructInCAN(const CANMessage &theDATA);
void canReadData(int packetSize);
void canReadExtRtr();
void BLEloop();
void slaveBTConnect(std::string name);
bool connectToServer(BLEAddress pAddress);
void readDATA();
void remplirStruct(int id, char len, char dt0, char dt1, char dt2, char dt3, char dt4, char dt5, char dt6, char dt7);


//----------------------------------------------------------------------callback fonctions
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    Serial.println((char*)pData);
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

//Callback function that gets called, when another device's advertisement has been received
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() == bleServerName) { //Check if the name of the advertiser matches
      advertisedDevice.getScan()->stop(); //Scan can be stopped, we found what we are looking for
      pServerAddress = new BLEAddress(advertisedDevice.getAddress()); //Address of advertiser is the one we need
      doConnect = true; //Set indicator, stating that we are ready to connect
      Serial.println("Device found. Connecting!");
    }
  }
};


static void CANNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  //store can value
  contenuBt = (char*)pData;
  newCan = true;
}


void Moteur_Init();
void setupPWM(int PWMpin, int PWMChannel);
void avancer(int vit);


//----------------------------------------------------------------------SETUP
void setup() {
  Serial.begin(115200);
  setupCAN();
  //Init BLE device
  slaveBTConnect("ESP32EA");
  Serial.printf("fin ble init\n");
  Encodeur_Init();
  Serial.printf("fin encodeur init\n");
  Moteur_Init();
  Serial.printf("fin moteur init\n");
  AsserInitCoefs();
  Asser_Init();

  init_Timer();
  Serial.printf("fin init Timer\n");

  DATArobot.ID = ALIVE_MOTEUR;
  DATArobot.RTR = true;
  //writeStructInCAN(DATArobot);
  Serial.printf("envoie CAN\n");

  Serial.println("fin setup\n");
}
//----------------------------------------------------------------------loop
void loop() {
  BLEloop();
  // Loop and read the mscount
	Odometrie();
  asser_position();

  if (mscount >= (TE_100US)) 
  {   
    DATArobot.ID = ERREUR_TEMP_CALCUL;
    DATArobot.ln = 2;
    DATArobot.dt[0] = mscount;
    DATArobot.dt[1] = TE_100US;
    //writeStructInCAN(DATArobot);
  }
  else 
  {
    while (mscount<(TE_100US));
  }
  mscount = 0;                    

}



//----------------------------------------------------------------------fonctions
/***************************************************************************************
 NOM : calcul                                                              
 ARGUMENT : aucun                                        
 RETOUR : rien                                                                        
 DESCRIPTIF :   Fonction principale appelé periodiquement, qui gère l'ensemble
                Se base sur : struct Ordre_deplacement liste[200];
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
                };
***************************************************************************************/

/***************************************************************************************
 NOM : Odometrie                                                                      
 ARGUMENT : rien                                                                      
 RETOUR : rien                                                                        
 DESCRIPTIF : Fonction qui calcul la position et l'orientation du robot               
***************************************************************************************/
void Odometrie(void)
{
    
    //Declaration des variables
    double dist, ang;   //distance, angle
    
    //Recuperation des valeurs des compteurs des encodeurs  
    Odo_val_pos_D = lireCodeurD();
    Odo_val_pos_G = lireCodeurG();
    erreur = Odo_val_pos_D - Odo_val_pos_G;
    //Calcul de la distance parcourue
    dist = 0.5*((Odo_val_pos_D - Odo_last_val_pos_D) + (Odo_val_pos_G - Odo_last_val_pos_G));
      
    //Calcul de la valeur de l'angle parcouru
    ang = ((Odo_val_pos_D - Odo_last_val_pos_D) -(Odo_val_pos_G - Odo_last_val_pos_G))*1800.0*PERIMETRE_ROUE_CODEUSE/(LARGEUR_ROBOT*M_PI*RESOLUTION_ROUE_CODEUSE);
    
    //Determination de la position sur le terrain en X, Y, Theta
    Odo_theta +=  ang;
    Odo_x += dist*cos((double)(Odo_theta*M_PI/1800.0))*PERIMETRE_ROUE_CODEUSE/RESOLUTION_ROUE_CODEUSE;
    Odo_y += dist*sin((double)(Odo_theta*M_PI/1800.0))*PERIMETRE_ROUE_CODEUSE/RESOLUTION_ROUE_CODEUSE;
    
    //Stockage de la derniere valeur de l'odometrie
    Odo_last_val_pos_D = Odo_val_pos_D;
    Odo_last_val_pos_G = Odo_val_pos_G;

    //mscount1 ++;
        
    /*/Condition d'envoi des informations de l'odometrie par CAN 
    if(mscount1 >= (500/TE_100US))
    {
      mscount1 = 0;
    }*/

    //Serial.printf("distance x : %lf et y : %lf ; angle : %lf\n", dist, ang, Odo_x, Odo_y, Odo_theta);

        
    
}  

void Moteur_Init(){
  //init pins carte de puiss
  pinMode(inApin_MOTG, OUTPUT);
  pinMode(inApin_MOTD, OUTPUT);
  pinMode(inBpin_MOTG, OUTPUT);
  pinMode(inBpin_MOTD, OUTPUT);
  pinMode(PWM_MOTG, OUTPUT);
  pinMode(PWM_MOTD, OUTPUT);

  //mise à l'arret
  digitalWrite(inApin_MOTG, LOW);
  digitalWrite(inApin_MOTD, LOW);
  digitalWrite(inBpin_MOTG, LOW);
  digitalWrite(inBpin_MOTD, LOW);

  //init pwm
  setupPWM(PWM_MOTD, PWMDChannel);
  setupPWM(PWM_MOTG, PWMGChannel);
}

//----------------------------------------------------------------------fonctions CAN et BLE

void BLEloop(){
  if(canAvailable){
    canAvailable = false;
    
    Serial.println("CAN received");
    canReadExtRtr();//On le me ici pour ne pas surcharger l'interruption CAN.onRecveive
    // Send message to master via bleutooth
    if (connected)
    {
      prxRemoteCharacteristic->writeValue((uint8_t *)&DATAtoSend, sizeof(DATAtoSend));
      Serial.println("Sending via BT...");
    }
    else{
      Serial.println("The device is not connected");
      
    }
  }

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer(*pServerAddress)) {
      Serial.println("We are now connected to the BLE Server.");
      //Activate the Notify property of each Characteristic
      ptxRemoteCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
      
    } else {
      Serial.println("We have failed to connect to the server; Restart your device to scan for nearby BLE server again.");
    }
    doConnect = false;
  }
  //if new CAN readings are available, write it in CAN bus
  if (newCan){
    newCan = false;
    readDATA();
    writeStructInCAN(myData); 
  }
  
}

void setupCAN(){
  while (!Serial);

  Serial.println("CAN Receiver ESP32-E-B");

  // start the CAN bus at 1000 kbps
  if (!CAN.begin(1000E3)) { //ici, nous avons modifié la bibliotheque CAN pour qu'elle soit compatible avec l'ESP32-E
    Serial.println("Starting CAN failed!");
    while (1);
  }
  CAN.onReceive(canReadData);
}

void writeStructInCAN(const CANMessage &theDATA){
  Serial.print("Sending ");

  if(theDATA.extented){
    CAN.beginExtendedPacket(theDATA.ID, theDATA.ln, theDATA.RTR);
    Serial.print("extended ");
  }
  else{CAN.beginPacket(theDATA.ID, theDATA.ln, theDATA.RTR);}

  Serial.print("packet on CAN... ");

  if(!theDATA.RTR){CAN.write(theDATA.dt, theDATA.ln);}
  
  CAN.endPacket();

  Serial.println("done");
  Serial.println();
}

void canReadData(int packetSize){
  
  DATAtoSend.ID = CAN.packetId();
  DATAtoSend.ln = CAN.packetDlc();
  // only print packet data for non-RTR packets
  int i = 0;
  while (CAN.available())
  {
    DATAtoSend.dt[i]=CAN.read();
    i++;
  }
  canAvailable = true;
}

void canReadExtRtr(){
  if (CAN.packetExtended()){
      Serial.print("extended ");
      DATAtoSend.extented = true;
  }else{DATAtoSend.extented = false;}
  if (CAN.packetRtr()){
      // Remote transmission request, packet contains no data
      Serial.print("RTR ");
      DATAtoSend.RTR = true;
  }
  else{DATAtoSend.RTR = false;}
}

void slaveBTConnect(std::string name){
  // Create the BLE Device
  BLEDevice::init(name);
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}
//Connect to the BLE Server that has the name, Service, and Characteristics
bool connectToServer(BLEAddress pAddress) {
   BLEClient* pClient = BLEDevice::createClient();
 
  // Connect to the remove BLE Server.
  pClient->connect(pAddress);
  Serial.println(" - Connected to server");
 
  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    return (false);
  }
 
  // Obtain a reference to the characteristics in the service of the remote BLE server.
  prxRemoteCharacteristic = pRemoteService->getCharacteristic(rxUUID);
  ptxRemoteCharacteristic = pRemoteService->getCharacteristic(txUUID);

  if (prxRemoteCharacteristic == nullptr || ptxRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID");
    return false;
  }
  Serial.println(" - Found our characteristics");
 
  //Assign callback functions for the Characteristics
  ptxRemoteCharacteristic->registerForNotify(CANNotifyCallback);
  connected = true;
  return true;
}

void readDATA(){
  //char contenuBt[sizeof(myData)]; // taille de la structure envoyée

  myData.extented = contenuBt[0];
  myData.RTR = contenuBt[1];
  myData.ID = contenuBt[4] + (contenuBt[5]<<8) + (contenuBt[6]<<16) + (contenuBt[7]<<24);
  myData.ln = contenuBt[8];
  int i;
  for(i=0;i<8;i++)
  {
    myData.dt[i]=contenuBt[i+9];
  }
}

void remplirStruct(int idf, char lenf, char dt0f, char dt1f, char dt2f, char dt3f, char dt4f, char dt5f, char dt6f, char dt7f){
  DATArobot.ID = idf;
  DATArobot.ln = idf;
  DATArobot.dt[0] = dt0f;
  DATArobot.dt[1] = dt1f;
  DATArobot.dt[2] = dt2f;
  DATArobot.dt[3] = dt3f;
  DATArobot.dt[4] = dt4f;
  DATArobot.dt[5] = dt5f;
  DATArobot.dt[6] = dt6f;
  DATArobot.dt[7] = dt7f;
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
  int freqMot = 20000;
  int resolution = 10;
  ledcSetup(PWMDChannel, freqMot, resolution);
  ledcSetup(PWMGChannel, freqMot, resolution);
  ledcAttachPin(PWM_MOTD, PWMDChannel);
  ledcAttachPin(PWM_MOTG, PWMGChannel);
}

/****************************************************************************************/
/* NOM : avancer                                                                        */
/* ARGUMENT : vitesse                                                                   */
/* RETOUR : rien                                                                        */
/* DESCRIPTIF : permet faire avancer le robot tout droit                                */
/****************************************************************************************/
void avancer(int vit)
{

  double erreur;
  double Kp = 0.95/100;
  erreur = lireCodeurD()-lireCodeurG();
  Serial.printf("erreur : %8f ; codeurD : %8f ; codeurG : %8f\n", erreur, lireCodeurD(), lireCodeurG());
  // moteur droit avancer
  digitalWrite(inBpin_MOTD, HIGH);
  digitalWrite(inApin_MOTD, LOW);

  // moteur gauche avancer
  digitalWrite(inBpin_MOTG, HIGH);
  digitalWrite(inApin_MOTG, LOW);

  ledcWrite(PWMDChannel, vit *(1-(Kp*erreur)));
  ledcWrite(PWMGChannel, vit +(1-(Kp*erreur)));
  

}





void test_accel(void)//fonctionne
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
        Serial.printf("accel test : %lf\n", accel_test);
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
        Serial.printf("accel test : %lf\n", accel_test);
        DATArobot.ID = ID_TEST_VITESSE;
        DATArobot.dt[0] = accel_test;
        DATArobot.ln = 1;
       // writeStructInCAN(DATArobot); 
        
        
        lposD = posD;
        lposG = posG;
        
        break;
    }
}



void asser_position(){
  double cmdD, cmdG, erreur;
  cmdD = Asser_Pos_MotD(0);
  cmdG = Asser_Pos_MotG(0);
  write_PWMD(cmdD);
  write_PWMG(cmdG);
  Serial.printf("PWMD : %lf; PWMG : %lf / ; / codeurD : %lf ; codeurG : %lf\n", cmdD, cmdG, lireCodeurD(), lireCodeurG());
}


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