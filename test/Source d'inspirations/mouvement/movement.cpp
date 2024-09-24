#include "movement.h"

using FuncType = std::function<float(float)>;

struct Ordre_deplacement liste;
bool finMvtElem = false, next_action = true;

float acc_max = 0.6;
float v_max = 0.8;
float ta;
float d_max;
int etat_automate_depl = INITIALISATION;

FuncType traj;
uint32_t start_time;

float kw = 3.f * 2;
float kv = 0.75f * 2;
float erreurPos = 0.07;

// Constants for PID control
const float Kp = 0.04; // Proportional gain
const float Ki = 0;    // Integral gain
const float Kd = 0.1; // Derivative gain

// Function to calculate PID control output
float calculatePID(float error, float dt)
{
    // Variables for PID control
    static float integral = 0.0, prev_error = 0.0;
    integral += error * dt;
    float derivative = (error - prev_error) / dt;
    float output = Kp * error + Ki * integral + Kd * derivative;
    prev_error = error;
    return output;
}


Position getSquareCoor(const MazeSquare *square, float squareSize)
{
    return getSquareCoor(square->i, square->j, squareSize);
}

Position getSquareCoor(uint8_t i, uint8_t j, float squareSize)
{
    Position coor;
    coor.x = (i + 0.5) * squareSize;
    coor.y = (j + 0.5) * squareSize;

    return coor;
}

float getDistance(const Position &p1, const Position &p2)
{
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}

double reductionAngle(double x)
{
    x = fmod(x + PI, 2 * PI);
    if (x < 0)
        x += 2 * PI;
    return x - PI;
}

FuncType fnVitesse2(Position init, Position fini)
{
    float distance = getDistance(init, fini);
    if (distance < d_max)
    {
        return [distance](float time)
        {
            float time_lim = sqrt(distance / acc_max);
            if (time < time_lim)
                return acc_max * time;
            else
                return v_max - acc_max * (time - time_lim);
        };
    }
    else
    {
        return [distance](float time)
        {
            float tc = (distance - d_max) / v_max;
            if (time < ta)
                return acc_max * time;
            else if (time >= ta && time <= tc + ta)
                return v_max;
            else
                return v_max - acc_max * (time - ta - tc);
        };
    }
}

float trajectoire(float time, FuncType velocityProfile)
{
    // Integrate velocity profile to get trajectory
    float trajectory = 0.0;
    for (float t = 0.0; t <= time; t += time)
    {
        trajectory += velocityProfile(t) * time;
    }

    return trajectory;
}

void positionControl(Position targetPos, float dt)
{
    const float Threshold = 0.01f, toleranceAngle = 8 * PI/180; // Adjust as needed
    static Position currentPos;
    float consvl = 0, consvr = 0;

    currentPos = gladiator->robot->getData().position;

    switch (etat_automate_depl)
    {
    case INITIALISATION:
    {
        ta = v_max / acc_max;
        d_max = v_max * v_max / acc_max;
        // currentPos = gladiator->robot->getData().position;
        traj = fnVitesse2(currentPos, targetPos);

        start_time = millis();
        etat_automate_depl = GO_TO_POS;
        float error = getDistance(currentPos, targetPos);
        if(error < Threshold){
            Serial.println("case INITIALISATION : etat_automate_depl = ROTATION");
            etat_automate_depl = ROTATION;
        }else{
            Serial.println("case INITIALISATION : etat_automate_depl = GO_TO_POS");
        }
        
    }
    break;
    case GO_TO_POS:
    {
        // Generate trajectory based on desired position
        float trajectory = trajectoire(dt, traj);
        
        float error = trajectory + getDistance(currentPos, targetPos);
        // gladiator->log("Position currentPos.x = %f, currentPos.y = %f, currentPos.a = %f, error = %f",currentPos.x, currentPos.y, currentPos.a, error);
        // gladiator->log("error = %f", error);
        float pidOutput = calculatePID(error, dt);
        // // gladiator->log("pidOutput = %f", pidOutput);
        
        // // gladiator->log("trajectory = %f", trajectory);
        // // Calculate final motor command
        // float motor_comman d = trajectory + pidOutput;
        // gladiator->log("motor_command = %f", motor_command);

        // Calculate consvl and consvr based on go_to function logic
        float dx = targetPos.x - currentPos.x;
        float dy = targetPos.y - currentPos.y;
        float d = sqrt(dx * dx + dy * dy);

        if (d > erreurPos)
        {
            float rho = atan2(dy, dx);
            float consw = kw * reductionAngle(rho - currentPos.a);

            float consv = kv * d * cos(reductionAngle(rho - currentPos.a));
            // consw = abs(consw) > wlimit ? (consw > 0 ? 1 : -1) * wlimit : consw;
            // consv = abs(consv) > vlimit ? (consv > 0 ? 1 : -1) * vlimit : consv;

            // Check if moving backward is more efficient
            float angleDifference = reductionAngle(rho - currentPos.a);
            if (abs(angleDifference) > PI / 2.)
            {
                gladiator->log("angleDifference : %f >PI/2, dt : %f", angleDifference, dt);
                consv = -consv; // Move backward if turning more than 90 degrees
            }

            consvl = consv - gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
            consvr = consv + gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
        }
        else
        {
            consvr = 0;
            consvl = 0;
        }

        if ((error < Threshold) || ((consvl + consvr) == 0))
        {
            // Serial.println("case GO_TO_POS : etat_automate_depl = ROTATION");
            // etat_automate_depl = ROTATION;
            Serial.println("case GO_TO_POS : etat_automate_depl = ARRET");
            etat_automate_depl = ARRET;
        }

        // Apply the PID correction
        consvl += pidOutput;
        consvr += pidOutput;
    }
    break;
    case ROTATION: {
            float Ka = 1;
            // currentPos = gladiator->robot->getData().position;
            // Calculate the angular difference between the target and current orientation
            float angleDifference = reductionAngle(targetPos.a - currentPos.a);
            float correctionSpeed = Ka * calculatePID(angleDifference, dt);

            // float pidOutput = calculatePID(angleDifference, dt);
            // Apply the correction speeds to the wheels
            consvl = correctionSpeed;
            consvr = -correctionSpeed;

            gladiator->log("angleDifference : %f,dt : %f, correctionSpeed : %f, consvl : %f, consvr : %f", angleDifference, dt, correctionSpeed, consvl, consvr);
            // consvl += pidOutput;
            // consvr -= pidOutput;

            // Check if the correction is complete
            if (abs(angleDifference) <= toleranceAngle) {
                gladiator->log("case ROTATION : etat_automate_depl = ARRET, toleranceAngle : %f", toleranceAngle);
                etat_automate_depl = ARRET;
                consvr = 0;
                consvl = 0;
            }
        }
        break;
    case ARRET:
    {
        gladiator->log("case ARRET: finMvtElem %d\n", finMvtElem);
        finMvtElem = true;
        etat_automate_depl = INITIALISATION;
        consvr = 0;
        consvl = 0;
    }
    break;
    default:
        break;
    }

    // Set wheel speeds based on the calculated motor commands
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);
    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false);
}

void calcul(void)
{
    // static int cpt_stop = 0;
    // static char etat_prec = TYPE_DEPLACEMENT_IMMOBILE, etat_automate_depl_prec = INITIALISATION;

    // float cmdD, cmdG;
    switch (liste.type)
    {
    case (TYPE_END_GAME):
    {
        // Arret
        break;
    }
    case (TYPE_DEPLACEMENT_IMMOBILE):
    {
        // cmdD = Asser_Pos_MotD(roue_drt_init);
        // cmdG = Asser_Pos_MotG(roue_gch_init);
        // write_PWMD(cmdD);
        // write_PWMG(cmdG);

        break;
    }
    case (TYPE_DEPLACEMENT_LIGNE_DROITE):
    {

        float dt = (millis() - start_time) * 0.0001f;
        positionControl(liste.fin, dt);
        if (finMvtElem)
        {
            Serial.println("Fin mouvement TYPE_DEPLACEMENT_LIGNE_DROITE");
            liste.type = (TYPE_MOUVEMENT_SUIVANT);
            finMvtElem = false;
            next_action = true;
        }
        break;
    }
    case (TYPE_DEPLACEMENT_ROTATION):
    {
        float dt = (millis() - start_time) * 0.0001f;
        positionControl(liste.fin, dt);
        if (finMvtElem)
        {
            Serial.println("Fin mouvement TYPE_DEPLACEMENT_ROTATION");
            liste.type = (TYPE_MOUVEMENT_SUIVANT);
            finMvtElem = false;
            next_action = true;
        }
        break;
    }
    case (TYPE_DEPLACEMENT_X_Y_THETA):
    {
        // X_Y_Theta(liste.x, liste.y, liste.theta, liste.sens, VMAX, AMAX);
        // if (finXYT){
        //     liste.type = (TYPE_MOUVEMENT_SUIVANT);
        //     finXYT = 0;
        //     next_action = true;
        //     //remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 2, (Message_Fin_Mouvement&0xFF), ((Message_Fin_Mouvement>>8)&0xFF),0,0,0,0,0,0);
        //     //writeStructInCAN(DATArobot);

        // }
        break;
    }
    case (TYPE_DEPLACEMENT_RAYON_COURBURE):
    {
        // Rayon_De_Courbure(liste.rayon, liste.theta_ray, VMAX, AMAX, liste.sens, DMAX);
        // if (finRayonCourbure){
        //     liste.type = (TYPE_MOUVEMENT_SUIVANT);
        //     finRayonCourbure = 0;
        //     next_action = true;
        //     //remplirStruct(DATArobot,INSTRUCTION_END_MOTEUR, 2, (Message_Fin_Mouvement&0xFF), ((Message_Fin_Mouvement>>8)&0xFF),0,0,0,0,0,0);
        //     //writeStructInCAN(DATArobot);
        // }
        break;
    }

    case TYPE_INIT_STOP:
    {
        liste.type = TYPE_STOP;
        break;
    }
    case TYPE_STOP:
    {
        break;
    }
    default:
    {
        break;
    }
    }

    if (liste.type == TYPE_MOUVEMENT_SUIVANT)
    {

        if (liste.enchainement == 1)
        {
        }
        else
        {
            // Remise a zero des variables
            //  consigne_vit = 0;
            //  consigne_pos = 0;
            // Reinitialisation etat automate des mouvements
            etat_automate_depl = INITIALISATION;
            // finMvtElem = 0;
            // On conserve la position du robot pour la prochaine action
            //  roue_drt_init = lireCodeurD();
            //  roue_gch_init = lireCodeurG();

            // On vide le buffer de mouvements et on prévoit de s'asservir sur la position atteinte
            liste.type = TYPE_DEPLACEMENT_IMMOBILE;
            // nb_ordres = 0;
            // cpt_ordre = 0;
            // cpt = 0;
        }
    }
}

bool TempsEchantionnage(uint16_t TIME)
{ // ms
    static uint32_t LastMscount = millis();
    if ((millis() - LastMscount) >= TIME)
    {
        // gladiator->log("erreur temp calcul %d", (int)(millis() - LastMscount));
        LastMscount = millis();
        return true;
    }
    else
    {
        return false;
    }
    
}