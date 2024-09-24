#include "math.h"
#include "guardConditions.h"
#define dist_thresh 1.7
using namespace std;

float distEucl(int ia, int ja, int ib, int jb)
{
    double dx = ia - ib;
    double dy = ja - jb;
    return sqrt(dx * dx + dy * dy);
}

bool ennemi_proche(Gladiator *gladiator)
{
    bool near = false;
    RobotData my_data = gladiator->robot->getData();

    RobotList ids_list = gladiator->game->getPlayingRobotsId();
    for (int i = 0; i < 4; i++)
    {

        if (ids_list.ids[i] != 121 && ids_list.ids[i] != 120)
        {
            RobotData others_data = gladiator->game->getOtherRobotData(ids_list.ids[i]);
            if (distEucl(my_data.position.x, my_data.position.y, others_data.position.x, my_data.position.y) < dist_thresh)
            {
                near = true;
            }
        }
    }
    return near;
}