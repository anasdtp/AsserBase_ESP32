#include "getCaseNeighboor.h"
#include <vector>

using namespace std;

int *getCaseNeighboor(int i, int j, GameState *game)
{

    static int arr[4] = {200, 200, 200, 200}; // 200= il y a un mur

    const MazeSquare *square;

    square = game->gladiator->maze->getSquare(i, j); // GFA 4.7.1

    // vérifier s'il a un mur au dessus
    if (square->northSquare == nullptr)
    {
        arr[0] = 200;
    }
    else
    {
        arr[0] = i + (j + 1) * 12;
    }
    // vérifier s'il a un mur à droite
    if (square->eastSquare == nullptr)
    {
        arr[1] = 200;
    }
    else
    {
        arr[1] = i + 1 + j * 12;
    }
    // vérifier s'il a un mur en dessous
    if (square->southSquare == nullptr)
    {
        arr[2] = 200;
    }
    else
    {
        arr[2] = i + (j - 1) * 12;
    }
    // vérifier s'il a un mur à gauche
    if (square->westSquare == nullptr)
    {
        arr[3] = 200;
    }
    else
    {
        arr[3] = i - 1 + j * 12;
    }

    return arr;
}
