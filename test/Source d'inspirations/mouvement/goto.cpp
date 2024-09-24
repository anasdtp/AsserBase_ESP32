#include "goto.h"
#include "Vianney/createPath.h"
#include "Vianney/creategraph.h"
#include <vector>
using namespace std;

bool followPath(GameState *game)
{
    Position current = game->gladiator->robot->getData().position; // game->myData.position;

    // go_to(game->goal, current, game->gladiator);

    if (getDistance(current, game->goal) <= THRESHOLD && game->count < game->simplified_coord_list.size)
    {
        game->goal = getSquareCoor(game->simplified_coord_list.path_coord[game->count].i, game->simplified_coord_list.path_coord[game->count].j, game->squareSize);
        game->gladiator->log("i: %d | j: %d", game->simplified_coord_list.path_coord[game->count].i, game->simplified_coord_list.path_coord[game->count].j);
        game->count++;

        liste.type = TYPE_DEPLACEMENT_LIGNE_DROITE;
        liste.fin = game->goal;
        gladiator->log("followPath(GameState *game) : Position liste.fin.x = %f, liste.fin.y = %f, liste.fin.a = %f", liste.fin.x, liste.fin.y, liste.fin.a);
        next_action = false;
        return true;
    }
    return false;
}

void new_missile(GameState *game)
{
    std::vector<int> path = BFS(game);
    game->coord_list.size = path.size();
    for (int i = 0; i < game->coord_list.size; i++)
    {
        game->coord_list.path_coord[i].i = path[i] % 12;
        game->coord_list.path_coord[i].j = path[i] / 12;
    }
    game->simplified_coord_list = createCommands(game->coord_list);
}