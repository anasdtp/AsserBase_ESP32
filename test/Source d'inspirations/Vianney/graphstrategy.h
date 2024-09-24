#pragma once
#include "getcaseneighboor.h"
#include <vector>
#include "GameData/GameData.h"

std::vector<int> BFSPruned(GameState *game);
int heuristic(const MazeSquare *sqr, GameState *game);
