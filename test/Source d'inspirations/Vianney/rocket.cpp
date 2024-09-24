#include "rocket.h"
// #include "gamedata.h"
using namespace std;

void getTarget()
{
    uint32_t start_time = millis() / 1000;

    while (millis() / 1000 - start_time < 1)
    {
        float dx = 1;
        float dy = 1;
        double alpha = atan2(dy, dx);
    }
}
