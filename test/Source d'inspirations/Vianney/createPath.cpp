#include "creategraph.h"
#include "createPath.h"
using namespace std;

pathFinder createCommands(pathFinder coord_list) // crée une suite d'instructions pour le robot en fonction d'une suite de cases à visiter
{
    pathFinder coord_list_simplified;
    coord_list_simplified.path_coord[0] = coord_list.path_coord[0];

    int count = 1;
    for (int i = 1; i < coord_list.size - 1; i++)
    {
        if (coord_list.path_coord[i].i != coord_list.path_coord[i - 1].i || coord_list.path_coord[i].i != coord_list.path_coord[i + 1].i) {
            if (coord_list.path_coord[i].j != coord_list.path_coord[i - 1].j || coord_list.path_coord[i].j != coord_list.path_coord[i + 1].j) {
                coord_list_simplified.path_coord[count] = coord_list.path_coord[i];
                ++count;
            }
        }
    }
    coord_list_simplified.path_coord[count] = coord_list.path_coord[coord_list.size - 1];
    coord_list_simplified.size = count + 1;
    return coord_list_simplified;
}
