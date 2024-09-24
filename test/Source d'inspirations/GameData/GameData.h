#pragma once
#include "gladiator.h"
#define max_parth_finder_size 24

struct coordonnees
{
    int i;
    int j;
};

struct pathFinder
{
    int size;
    coordonnees path_coord[max_parth_finder_size];
};

class GameState
{
public:
    Gladiator *gladiator;
    RobotData myData;
    RobotData allyData;
    RobotData er1Data;
    RobotData er2Data;

    Position goal;

    float squareSize;

    uint8_t count;
    pathFinder coord_list;
    pathFinder simplified_coord_list;

    GameState(Gladiator *gladiator);

    void Update();
    void reset();
};