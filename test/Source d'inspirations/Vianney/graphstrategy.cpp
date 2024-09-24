#include <iostream>
#include <queue>
#include <vector>
#include <unordered_set>
#include <tuple>
#include "getcaseneighboor.h"
#include "graphstrategy.h"
#include <unordered_map>
#include <algorithm>

using namespace std;

vector<int> BFSPruned(GameState *game)
{
    // Marquer tous les sommets comme non visités
    bool visited[144] = {false}; // les coordonnées des sommets sont codées en un int a, tq : a=i+j*12

    const MazeSquare *square = game->gladiator->maze->getNearestSquare(); // position du robot comme case de départ

    uint8_t i_index = square->i; // indice colonne selon la convention
    uint8_t j_index = square->j; // indice ligne selon la convention

    int start_coord = i_index + j_index * 12;
    int end_coord;

    queue<int> q; // Créer une file pour le BFS

    visited[start_coord] = true; // Marquer le sommet de départ comme visité et l'ajouter à la file
    q.push(start_coord);
    unordered_map<int, tuple<int, int>> path_dict; // key,pred,cost
    int depth_goal = 50;                           // nombre de noeud à parcourir
    int depth = 1;
    int index_min;
    int min = 500;
    while (depth < depth_goal)
    {
        // Extraire un sommet de la file
        int currentVertex = q.front();
        int i = currentVertex % 12;
        int j = currentVertex / 12;
        q.pop();
        int *result = getCaseNeighboor(i, j, game);

        for (int i = 0; i < 4; ++i) // Parcourir tous les sommets adjacents du sommet courant
        {

            int adjacentVertex = result[i];
            int i_a = adjacentVertex % 12;
            int j_a = adjacentVertex / 12;
            game->gladiator->log("case visitée :%d,%d", i_a, j_a);
            // Si un sommet adjacent n'a pas encore été visité, le marquer comme visité
            // et l'ajouter à la file
            if (adjacentVertex != 200)
            {
                if (!visited[adjacentVertex])
                {
                    visited[adjacentVertex] = true;
                    q.push(adjacentVertex);
                    tuple<int, int> t;
                    get<0>(t) = currentVertex;
                    const MazeSquare *sqr = game->gladiator->maze->getSquare(i_a, j_a);
                    int h = heuristic(sqr, game); // on calcule les heuristiques lorsqu'on regarde les noeuds voisins sans les visiter
                    get<1>(t) = h;
                    path_dict[adjacentVertex] = t;
                    if (h < min)
                    {
                        index_min = adjacentVertex;
                    }
                }
            }
        }
        depth++;
    }
    // trouve la case de plus faible cout
    end_coord = index_min;
    // trouve le chemin vers la case de plus faible cout
    vector<int> path;
    int current = end_coord;
    while (current != start_coord)
    {
        path.push_back(current);
        current = get<0>(path_dict.at(current));
    }
    path.push_back(start_coord);
    reverse(path.begin(), path.end()); // Inverser le chemin car on l'a construit de la fin au début
    return path;
}

int heuristic(const MazeSquare *sqr, GameState *game)
{
    // si c'est proche du bord
    int h = 0;
    uint32_t time = millis() / 1000; // temps en secondes
    int i = sqr->i;
    int j = sqr->j;
    game->gladiator->log("TIME %d : ", time);

    uint32_t time_thresh_init = 8; // temps à partir du quel il faut faire attention au shr
    uint32_t time_between_shrinking = 4;
    if (time > time_thresh_init)
    {
        int shrink_progress = (time - 8) / time_between_shrinking;
        if (i <= shrink_progress || (12 - i) <= shrink_progress || j <= shrink_progress || (12 - j) <= shrink_progress)
        {
            game->gladiator->log("case a éviter en %d,%d", i, j);
            h += 500;
        }
    }
    return h;
}