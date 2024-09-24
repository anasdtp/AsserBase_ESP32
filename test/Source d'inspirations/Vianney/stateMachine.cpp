#include "stateMachine.h"
#include "GameData/GameData.h"
#include <vector>
#include "mouvement/movement.h"
#include "mouvement/goto.h"
StateMachine::StateMachine(GameState *game) : game(game), currentState(State::ATTENTE) {}

void StateMachine::transition()
{
    bool t_recherche_fusee = !game->gladiator->weapon->canLaunchRocket();
    bool t_ennemi_proche = ennemi_proche(game->gladiator);
    bool t_recherche_cible = true;
    bool t_tirer = true;
    game->gladiator->log("void StateMachine::transition() : Possède une fusée : %d", t_recherche_fusee);

    switch (currentState)
    {
    case State::ATTENTE:
        if (t_ennemi_proche)
        {
            currentState = State::PVP;
        }
        else if (t_recherche_fusee)
        {
            currentState = State::RECHERCHE_FUSEE;
        }
        else
        {
            currentState = State::EXPLORATION;
        }
        break;

    case State::RECHERCHE_FUSEE:
        new_missile(game);
        currentState = State::ATTENTE;

        break;

    case State::EXPLORATION:
    {
        static int etat = 0;
        switch (etat)
        {
        case 0:
        {
            game->count = 0;
            std::vector<int> path = BFSPruned(game);
            if(followPath(game)){
                etat = 1;
            }
        }
        break;
        case 1 :
        {
            if(next_action){
                next_action = false;
                etat = 0;
            }
        }
        break;
        default:
            break;
        }
        
        if (t_recherche_cible)
        {
            currentState = State::RECHERCHE_CIBLE;
        }
        if (t_ennemi_proche)
        {
            currentState = State::PVP;
        }
        else
        {
            currentState = State::ATTENTE;
        }
    }
    break;

    case State::PVP:
        executePVP();
        currentState = State::ATTENTE;
        break;

    case State::RECHERCHE_CIBLE:
        t_recherche_cible = false;
        if (t_tirer)
        {
            currentState = State::TIRER;
        }
        else
        {
            currentState = State::EXPLORATION;
        }
        break;

    case State::TIRER:
        game->gladiator->weapon->launchRocket();
        currentState = State::ATTENTE;
        break;
    }
}
void StateMachine::executePVP()
{
    RobotData my_data = game->gladiator->robot->getData();
    RobotData ally_data;
    RobotList ids_list = game->gladiator->game->getPlayingRobotsId();
    for (int i = 0; i < 4; i++)
    {
        if ((ids_list.ids[i] == 121 || ids_list.ids[i] != 120) && ids_list.ids[i] != my_data.id)
        {
            ally_data = game->gladiator->game->getOtherRobotData(ids_list.ids[i]);
        }
    }
    SpartanMode(game);
}
