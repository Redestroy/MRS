#include <cstdio>
#include <iostream>
#include <rcsc/common/basic_client.h>
#include "RCSCSoccerRobot.h"
#include <MRS/Include/Behaviour.hpp>
#include <MRS/Include/ATask.hpp>
#include <MRS/Include/ConditionNull.hpp>
#include <MRS/Include/ParametricATask.hpp>
#include <MRS/Include/PredicateCondition.hpp>
#include <MRS/Include/AbsoluteTimeCondition.hpp>

#include <iostream>
#include <cstdlib> // exit
#include <cerrno> // errno
#include <cstring> // strerror
#include <csignal> // sigaction

MRS::RCSCSoccerRobot robot;

//Code from agent2d::main_player 
void sig_exit_handle(int)
{
    std::cerr << "Killed. Exiting..." << std::endl;
    robot.Exit();
    robot.finalize();
    std::exit(EXIT_FAILURE);
}


int main(int argc, char* argv[])
{
    //Code from agent2d::main_player 
    struct sigaction sig_action;
    sig_action.sa_handler = &sig_exit_handle;
    sig_action.sa_flags = 0;
    if (sigaction(SIGINT, &sig_action, NULL) != 0
        || sigaction(SIGTERM, &sig_action, NULL) != 0
        || sigaction(SIGHUP, &sig_action, NULL) != 0)
        /*if ( signal(SIGINT, &sigExitHandle) == SIG_ERR
          || signal(SIGTERM, &sigExitHandle) == SIG_ERR
          || signal(SIGHUP, &sigExitHandle) == SIG_ERR )*/
    {
        std::cerr << __FILE__ << ": " << __LINE__
            << ": could not set signal handler: "
            << std::strerror(errno) << std::endl;
        std::exit(EXIT_FAILURE);
    }

    rcsc::BasicClient client;
    if (!robot.init(&client, argc, argv))
    {
        return EXIT_FAILURE;
    }


    std::cout << "*****************************************************************\n"
              << " This program is modified by LMinc Robotics\n"
              << " Copyright 2021. Jānis Bremanis.\n"
              << " Riga Technical University\n"
              << " All rights reserved.\n"
              << "*****************************************************************\n"
              << std::flush;

    /*
        Do NOT remove the following copyright notice!
    */
    std::cout << "*****************************************************************\n"
        << " This program is based on agent2d created by Hidehisa Akiyama.\n"
        << " Copyright 2006 - 2011. Hidehisa Akiyama and Hiroki Shimora.\n"
        << " All rights reserved.\n"
        << "*****************************************************************\n"
        << std::flush;

    // Create behaviours
    MRS::Task::Behaviour waiting_behaviour(1, 0.1f);
    MRS::Task::Behaviour waiting_behaviour_with_condition(2, 0.1f);
    // Create and assign values to conditions
    MRS::Task::AbsoluteTimeCondition start_time(3000000000,true);
    MRS::Task::PredicateCondition game_phase_is_before_kick_off("game_phase_is_before_kick_off",true);
    MRS::Task::PredicateCondition game_phase_is_play_game("game_phase_is_play_game", true);
    MRS::Task::PredicateCondition game_phase_is_after_game("game_phase_is_after_game", true);
    MRS::Task::ConditionNull null_condition;
    // Create and assign values to base tasks
    MRS::Task::ATask base_task(1,3,0.05f, &null_condition, &null_condition);
    base_task.Add(MRS::Device::Action());
    MRS::Task::ParametricATask base_task2;

    // Assign values to behaviours
    // Behaviour 1: time based waiting behaviour
    waiting_behaviour.SetTimestamp(0);
    waiting_behaviour.SetState(MRS::Task::TaskState::NEW);
    waiting_behaviour.SetBehaviorType(MRS::Task::BehaviourType::WAITING_BEHAVIOUR);
    waiting_behaviour.SetEndCondition(&start_time);
    waiting_behaviour.SetBaseTask(&base_task);

    // Behaviour 2: condition based waiting behaviour
    waiting_behaviour_with_condition.SetTimestamp(0);
    waiting_behaviour_with_condition.SetState(MRS::Task::TaskState::NEW);
    waiting_behaviour_with_condition.SetBehaviorType(MRS::Task::BehaviourType::WAITING_BEHAVIOUR);
    waiting_behaviour_with_condition.SetEndCondition(&game_phase_is_play_game);
    waiting_behaviour_with_condition.SetBaseTask(&base_task);

    // Behaviour 3: standby behaviour
    // after b1 and b2 build and work

    // Add behaviours to behaviour library
    robot.GetBehaviourLibrary()->Add("Waiting", &waiting_behaviour);
    robot.GetBehaviourLibrary()->Add("WaitingFor", &waiting_behaviour_with_condition);

    client.run(&robot);
    std::cout << "After run method " << std::endl;
    return EXIT_SUCCESS;

}