#pragma once
#define max_parth_finder_size 24
#include <vector>
#include "GameData/GameData.h"

std::vector<int> BFS(GameState *game, bool rocket_man = true, int i_goal = 0, int j_goal = 0);