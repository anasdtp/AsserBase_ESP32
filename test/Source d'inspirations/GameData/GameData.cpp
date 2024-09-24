#include "GameData.h"

GameState::GameState(Gladiator *gladiator){
    this->gladiator = gladiator;
    this->reset();
}

void GameState::Update(){
    myData = gladiator->robot->getData();
    allyData = gladiator->game->getOtherRobotData(allyData.id);
    er1Data = gladiator->game->getOtherRobotData(er1Data.id);
    er2Data = gladiator->game->getOtherRobotData(er2Data.id);
}

void GameState::reset(){
    myData = gladiator->robot->getData();

    bool enn_init = true;
    for(int i = 0; i < 4; ++i){
        byte id = gladiator->game->getPlayingRobotsId().ids[i];
        if(id == myData.id) continue;
        if(gladiator->game->getOtherRobotData(id).teamId == myData.teamId)
            allyData = gladiator->game->getOtherRobotData(id);
        else {
            if(enn_init) {
                er1Data = gladiator->game->getOtherRobotData(id);
                enn_init = false;
            } else {
                er2Data = gladiator->game->getOtherRobotData(id);
            }
        }
    }

    squareSize = gladiator->maze->getSquareSize();
}