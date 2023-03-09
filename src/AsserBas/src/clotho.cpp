#include <math.h>
#include "clotho.h"
#include "CRAC_utility.h"

#define N 1000
#define Amplitude 2.0
const double h = Amplitude/N;

static double T[N+1][3];


double Fonct1(double x)
{
    return cos(x * x);
}

double Fonct2(double x)
{
    return sin(x * x);
}

double I(double x)
{
    double ph = sqrt(x);
    int A1 = ph/h;
    return T[A1][1] + (ph - A1 * h) * (T[A1+1][1] - T[A1][1]) / h;
}

double J(double x)
{
    double ph = sqrt(x);
    int A1 = ph/h;
    return T[A1][2] + (ph - A1 * h) * (T[A1+1][2] - T[A1][2]) / h;
}


double Expression(double K, double Alpha, double R, double Phi, double E)
{
    double ph = sqrt(Phi);
    return 2 * K * ph * (I(Phi) * cos(Alpha / 2) + J(Phi) * sin(Alpha / 2))+ K * sin(Alpha / 2 - Phi)- 2 * ph * (E / 2 + R) * sin(Alpha / 2);
}


/*
Vit : vitesse initiale
Gamma : Acceleration/2
Alpha : Angle de rotation en Rads
R : Rayon de courbure
E: entrax du robot
K: Sqr(E / Gamma) * Vit

renvoie 0 si virage impossible si non 1
*/
double Cap(double Vit, double Gamma, double Alpha, double R, double E)
{
    double K = sqrt(E / Gamma) * Vit;
    double V = Alpha / 2;
    double u = 0;
    double m = (u + V) / 2;
    do
    {
        tC3 = mscount;nbexpr++;
        m = (u + V) / 2;
        if(Expression(K, Alpha, R, m, E) > 0)u = m;
            else V = m;
    } while((V - u) > 0.001);
    return m;
}


void calculTabRefClotho()
{
    int K;
    for (K = 1; K<=N; K++)
    {
        T[K][1] = 4 * Fonct1((K - 0.5) * h);
        T[K][2] = 4 * Fonct2((K - 0.5) * h);
    }

    for (K = 1; K<N; K++)
    {
        double x = Fonct1(K * h);
        T[K][1] = T[K][1] + x;
        T[K+1][1] = T[K+1][1] + x;
        x = Fonct2(K * h);
        T[K][2] = T[K][2] + x;
        T[K+1][2] = T[K+1][2] + x;
    }

    T[1][1] = T[1][1] + Fonct1(0);
    T[N][1] = T[N][1] + Fonct1(Amplitude);
    T[1][2] = T[1][2] + Fonct2(0);
    T[N][2] = T[N][2] + Fonct2(Amplitude);

    for (K = 1; K<=N; K++)
    {
        T[K][1] = T[K][1] * h / 6;
        T[K][2] = T[K][2] * h / 6;
    }

    for (K = 2; K<=N; K++)
    {
        T[K][1] = T[K][1] + T[K-1][1];
        T[K][2] = T[K][2] + T[K-1][2];
    }
}
/*
Vit : vitesse initiale
Gamma1 : Acceleration et deceleration
Alpha : Angle de rotation en Rads
R : Rayon de courbure
E: entrax du robot

*/
int possibiliteVirage(clothoStruc *clotho)
{
    double Gamma = clotho->accel / 2;
    double anglRad = (M_PI*clotho->angleDArc)/1800 ;
    double K = sqrt(clotho->E / Gamma) * clotho->vit;
    if(Expression(K, anglRad, clotho->rayondDeCourbre, clotho->angleDArc / 2, clotho->E)>0) return 0;
    else return 1;
}


void trajectoire(clothoStruc *clotho)
{
    tC1 = mscount;
    double Gamma = clotho->accel / 2;
    double anglRad = (M_PI*clotho->angleDArc)/1800 ;
    tC2 = mscount;
    nbexpr = 0;
    double Phi1 = Cap(clotho->vit, Gamma, anglRad, clotho->rayondDeCourbre, clotho->E);
    tC4 = mscount;
    clotho->tClotho = sqrt(clotho->E * Phi1 / Gamma);

    double Phi2 = anglRad - 2 * Phi1;
    clotho->tArc = clotho->E * Phi2 / Gamma / clotho->tClotho / 2;

    clotho->vMin = clotho->vit - clotho->accel * clotho->tClotho;

    //double TI = T1 + T2;
    clotho->tTotal = clotho->tArc + 2*clotho->tClotho;
    tC5 = mscount;
    
}

