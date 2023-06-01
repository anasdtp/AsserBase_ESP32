/****************************************************************************************/
/*                                   IDENT_CRAC.H                                     */
/****************************************************************************************/

/****************************************************************************************/
/*                          Securite contre les multi-inclusions                        */
/****************************************************************************************/
#ifndef CRAC_IDENTH
#define CRAC_IDENTH

/****************************************************************************************/
/*                              Identiants pour le bus CAN                              */
/****************************************************************************************/
#define GLOBAL_GAME_END 0x004  // Stop fin du match
#define GLOBAL_START 0x002  // Start
#define GLOBAL_END_INIT_POSITION 0x005  // Fin positionnement robot avant depart
#define GLOBAL_FUNNY_ACTION 0x007  // Funny action start  (0: start, 1: stop)

    
#define BALISE_STOP 0x003  // Trame stop  (angle en °, Type du robot : 1=>gros robot, 2=> petit)
#define BALISE_DANGER 0xA  // Trame danger  (angle en °, Type du robot : 1=>gros robot, 2=> petit)
#define BALISE_END_DANGER 0xB  // Trame fin de danger


#define ASSERVISSEMENT_STOP 0x001  // Stop moteur
#define ASSERVISSEMENT_SPEED_DANGER 0x006  // Vitesse de danger
#define ASSERVISSEMENT_XYT 0x020  // Asservissement (x,y,theta)  (0 : au choix 1 : avant -1 : arrière)
#define ASSERVISSEMENT_COURBURE 0x021  // Asservissement rayon de courbure  (+ gauche, - droite , sens : 1avt , -1arr; enchainement => 1 oui, 0 => non, 2=>derniére instruction de l'enchainement)
#define ASSERVISSEMENT_CONFIG 0x022  // Asservissement paramètre  (définir les valeurs de vitesse max et d'eccélération max)
#define ASSERVISSEMENT_ROTATION 0x023  // Asservissement rotation
#define ASSERVISSEMENT_RECALAGE 0x024  // Moteur tout droit  (recalage : 0 mouvement seul, 1 x, 2y valeur : coordonnée à laquelle est recalé x/y; enchainement => 1 oui, 0 => non)
#define ASSERVISSEMENT_DECEL 0x019 



    
#define ODOMETRIE_BIG_POSITION 0x026  // Odométrie position robot  (Position actuel du robot)
#define ODOMETRIE_BIG_VITESSE 0x027  // Odométrie vitesse  (Indication sur l'état actuel)
#define ODOMETRIE_SMALL_POSITION 0x028  // Odométrie position robot  (Position actuel du robot)
#define ODOMETRIE_SMALL_VITESSE 0x029  // Odométrie vitesse  (Indication sur l'état actuel)

#define ASSERVISSEMENT_INFO_CONSIGNE 0x1F0  // Info Consigne et Commande moteur
#define ASSERVISSEMENT_CONFIG_KPP_DROITE 0x1F1  // Config coef KPP_Droit
#define ASSERVISSEMENT_CONFIG_KPI_DROITE 0x1F2  // Config coef KPI_Droit
#define ASSERVISSEMENT_CONFIG_KPD_DROITE 0x1F3  // Config coef KPD_Droit
#define ASSERVISSEMENT_CONFIG_KPP_GAUCHE 0x1F4  // Config coef KPP_Gauche
#define ASSERVISSEMENT_CONFIG_KPI_GAUCHE 0x1F5  // Config coef KPI_Gauche
#define ASSERVISSEMENT_CONFIG_KPD_GAUCHE 0x1F6  // Config coef KPD_Gauche
#define ASSERVISSEMENT_CONFIG_PERIMETRE_ROUE_CODEUSE 0x1FC
#define ASSERVISSEMENT_CONFIG_LARGEUR_ROBOT 0x1FD
#define ASSERVISSEMENT_CONFIG_KPP 0x1F8  // Config coef KPP
#define ASSERVISSEMENT_CONFIG_KPI 0x1F9  // Config coef KPI
#define ASSERVISSEMENT_CONFIG_KPD 0x1FA  // Config coef KPD
#define ASSERVISSEMENT_ENABLE 0x1F7  // Activation asservissement  (0 : désactivation, 1 : activation)
#define ASSERVISSEMENT_REQUETE_PID 0x1FB


#define ASSERVISSEMENT_CONFIG_KPP_Qt 0x0F0  // Config coef KPP
#define ASSERVISSEMENT_CONFIG_KPI_Qt 0x0F1  // Config coef KPI
#define ASSERVISSEMENT_CONFIG_KPD_Qt 0x0F2  // Config coef KPD
#define ASSERVISSEMENT_CONFIG_LARGEUR_ROBOT_Qt 0x0F3  
#define ASSERVISSEMENT_CONFIG_PERIMETRE_ROUE_CODEUSE_Qt 0x0F4 
#define ASSERVISSEMENT_ROTATION_Qt   0x0F5  // Asservissement rotation
#define ASSERVISSEMENT_RECALAGE_Qt   0x0F6
#define ASSERVISSEMENT_XYT_Qt        0x0F7
#define ASSERVISSEMENT_COURBURE_Qt   0x0F8 


    
#define ASSERVISSEMENT_DECEL 0x019 
#define ASSERVISSEMENT_BEZIER 0x02A

#define RESET_BALISE 0x030  // Reset balise
#define RESET_MOTEUR 0x031  // Reset moteur
#define RESET_IHM 0x032  // Reset écran tactile
#define RESET_ACTIONNEURS 0x033  // Reset actionneurs
#define ESP32_RESTART 0x34


#define RESET_STRAT 0x3A  // Reset stratégie

#define CHECK_BALISE 0x060  // Check balise
#define CHECK_MOTEUR 0x061  // Check moteur
#define CHECK_IHM 0x062  // Check écran tactile
#define CHECK_ACTIONNEURS 0x063  // Check actionneurs


#define ALIVE_BALISE 0x070  // Alive balise
#define ALIVE_MOTEUR 0x071  // Alive moteur
#define ALIVE_IHM 0x072  // Alive écran tactile
#define ALIVE_ACTIONNEURS 0x073  // Alive actionneurs


#define ACKNOWLEDGE_BALISE 0x100  // Acknowledge balise
#define ACKNOWLEDGE_MOTEUR 0x101  // Acknowledge moteur
#define ACKNOWLEDGE_IHM 0x102  // Acknowledge ecran tactile
#define ACKNOWLEDGE_ACTIONNEURS 0x103  // Acknowledge actionneurs
#define ACKNOWLEDGE_BEZIER 0x104 //Acknowledge bezier


#define INSTRUCTION_END_BALISE 0x110  // Fin instruction balise  (Indique que l'instruction est terminée)
#define INSTRUCTION_END_MOTEUR 0x111  // Fin instruction moteur  (Indique que l'instruction est terminée)
#define INSTRUCTION_END_IHM 0x112  // Fin instruction ecran tactile  (Indique que l'instruction est terminée)
#define INSTRUCTION_END_ACTIONNEURS 0x113  // Fin instruction actionneurs  (Indique que l'instruction est terminée)


#define ECRAN_CHOICE_STRAT 0x601  // Choix d'une stratégie  (n° strat (1-4))
#define ECRAN_CHOICE_COLOR 0x602  // Couleur  (0->Purple;1->green)
#define ECRAN_START_MATCH 0x603  // Match  (Indique que l'on souhaite commencer le match)
#define ECRAN_ACK_STRAT 0x611  // Acknowledge stratégie  (si 0 erreur, sinon n°strat)
#define ECRAN_ACK_COLOR 0x612  // Acknowledge couleur  (0->Purple;1->green)
#define ECRAN_ACK_START_MATCH 0x613  // Acknowledge Match  (Indique que l'on a bien reçu le debut du match)
#define ECRAN_ALL_CHECK 0x620  // Carte all check  (Si provient de carte strat => toutes les cartes sont en ligne, Si provient IHM => forcer le lancement)
#define ECRAN_TIME 0x621  // Time match  (Indication de moment cle du temps (10,30,60,70,80,85,90))
#define ECRAN_PRINTF_1 0x6C0  // Tactile printf  (Afficher les 8 permier caractères)
#define ECRAN_PRINTF_2 0x6C1  // Tactile printf  (Afficher les 8 second caractères)
#define ECRAN_PRINTF_3 0x6C2  // Tactile printf  (Afficher les 8 troisième caractères)
#define ECRAN_PRINTF_4 0x6C3  // Tactile printf  (Afficher les 8 quatrième caractères)
#define ECRAN_PRINTF_CLEAR 0x6CF  // Tactile printf clear  (Permet d'effacer l'ecran)


#define ERROR_OVERFLOW_BALISE 0x040  // Overflow odométrie
#define ERROR_OVERFLOW_MOTEUR 0x041  // Overflow asservissement
#define ERROR_OVERFLOW_IHM 0x042  // Overflow balise
#define ERROR_OVERFLOW_STRAT 0x043  // Overflow stratégie
#define ERROR_BALISE 0x785  // Bug balise
#define ERROR_RTC 0x786  // Bug RTC
#define ERROR_MOTEUR 0x787  // Bug moteur
#define ERROR_TELEMETRIE 0x788  // Bug télémètre
#define ERROR_STRATEGIE 0x789  // Bug stratégie

    
#define DEBUG_STRATEGIE_AUTOMATE 0x760  // Etat automate stratégie  (Permet de savoir l'etat de l'automate)
#define DEBUG_FAKE_JAKE 0x761  // Fake jack  (Permet d'outre passerr le JACk du robot)
#define DEBUG_ASSERV 0x762  // Info debug carte moteur

    
#define SERVO_AX12_SETGOAL 0x090  // AX12 setGoal  (Indiquer la nouvelle position de l'AX12 !! Ne bouge pas)
#define SERVO_AX12_PROCESS 0x091  // AX12 processChange  (Lancer le déplacement des AX12)
#define SERVO_AX12_DONE 0x092  // AX12 done  (Indique q'un AX12 a terminé son déplacement)
#define SERVO_XL320 0x093  // XL320
    
#define DEBUG_MS_COUNT 0x055   
    
    
       
#define ASSERVISSEMENT_ERREUR 0x025    
    
#define ID_FIN_CLOTHO 0x501

    

    
#define ID_ENTRAXE 0x510
#define ID_RAYON 0x511
#define ID_ALPHA 0x512
#define ID_VITESSE 0x513
#define ID_ACCELERATION 0x514
#define ID_TCLOTHO 0x515
#define ID_TARC 0x516
#define ID_TEMPS 0x517
#define ID_VIT 0x518
#define ID_VIT1 0x519
#define ID_POS 0x520
#define ID_POS1 0x521
#define ID_T_CALCUL 0x522

    
#define ERREUR_TEMP_CALCUL 0x5A0

#define ID_DBUG_ETAT 0x5A1
#define ID_DBUG_ETAT_DPL 0x5A2
    
    
#define ID_DBUG_LIGNE_TPS 0x5A3
#define ID_DBUG_LIGNE_PCONS 0x5A4
#define ID_DBUG_LIGNE_VIT 0x5A5
#define ID_DIST_TIC_GENE 0x5A6  
#define ID_TEMPS_CALCUL_CLOTHO 0x5A7
#define ID_DBUG_LIGNE_GENE_VIT 0x5A8
#define ID_CLOTHO_IMPOSSIBLE 0x5A9
    
#define ID_TRAIT_LIGNE_GENE 0x5C0
#define ID_TRAIT_CLOTHO 0x5C1
#define ID_TRAIT 0x5C2
    
#define ID_TEMPS_LONG_1 0x5C3
#define ID_TEMPS_LONG_2 0x5C4
    
#define ID_TEST_VITESSE 0x5C5

/****************************************************************************************/

/****************************************************************************************/
/*                              Identiants pour le bus CAN Lidar                        */
/****************************************************************************************/

#define IDCAN_POS_XY_OBJET 0x82
#define ERREUR_TEMP_CALCUL_LIDAR 0x86
    
#endif
