#include "melee.h"

bool AttackPlayer(RobotData player, RobotData enemy){
    if(enemy.lifes <= 0) return false;
    Position current = player.position;
    Position er1 = enemy.position;
    float dx = current.x - er1.x;
    float dy = current.y - er1.y;

    float distance = sqrt(dx*dx + dy*dy);

    if(distance < ATTACK_DISTANCE){
        float orientation = atan2(dy, dx);

        if(er1.a >= -orientation - PI/2 && er1.a <= -orientation + PI/2){
            return true;
        }
    }

    return false;
}

void SpartanMode(GameState *game){
    if(AttackPlayer(game->myData, game->er1Data)){
        game->goal = game->er1Data.position;
    } else if(AttackPlayer(game->myData, game->er2Data)){
        game->goal = game->er2Data.position;
    }
    else{
        game->gladiator->control->setWheelSpeed(WheelAxis::LEFT, 1);
        game->gladiator->control->setWheelSpeed(WheelAxis::RIGHT, -1);
    }
}