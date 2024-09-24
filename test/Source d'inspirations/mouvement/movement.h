#pragma once
#include "gladiator.h"
#include <functional>

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
#define GO_TO_POS 1
#define ROTATION 4
#define ARRET 3



#define TE_10MS 50
#define TE (TE_10MS*0.0001)


#define THRESHOLD 0.07

extern Gladiator* gladiator;

/****************************************************************************************/
/*                          Definition de la structure deplacement                      */
/****************************************************************************************/
struct Ordre_deplacement
{
    char type, enchainement;
    Position init, fin;
};

extern float kw, kv, wlimit, vlimit, erreurPos;


extern float acc_max, v_max, ta, d_max;
extern int etat_automate_depl;

extern struct Ordre_deplacement liste;
extern bool finMvtElem, next_action;




Position getSquareCoor(const MazeSquare *square, float squareSize);
Position getSquareCoor(uint8_t i, uint8_t j, float squareSize);
float getDistance(const Position &p1, const Position &p2);
double reductionAngle(double x);

void calcul();
void positionControl(Position targetPos, double dt);
bool TempsEchantionnage(uint16_t TIME);
