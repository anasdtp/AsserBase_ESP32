#include <Arduino.h>
#define CLOTHO_H_INCLUDED

//typedef struct clothoStruc clothoStruct;
    
    
/* les unités doivent être equivalentes
dans notre cas :
    
    distance en tic
    temps en TE (periodes d'automate)
    angles en dixième de degré
    
    vitesse tic/TE
    accélérations tic/TE²
    */
typedef struct {
    double E;
    double accel;
    double vit;

    double angleDArc;
    double rayondDeCourbre;

    double tClotho;
    double tArc;
    double tTotal;
    double vMax;
    double vMin;
}clothoStruc;


double Fonct1(double x);
double Fonct2(double x);
double I(double x);
double J(double x);
double Expression(double K, double Alpha, double R, double Phi, double E);
double Cap(double Vit, double Gamma, double Alpha, double R, double E);
void calculTabRefClotho();

void trajectoire(clothoStruc *clotho);
int possibiliteVirage(clothoStruc *clotho);

/* [] END OF FILE */
