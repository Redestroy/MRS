#include <cstdio>
#include "../MRS_Algorithms_cpp/Include/MRS_SCA.hpp"
int main()
{
    printf("hello from MRS_Algorithms_Testing!\n");
    printf("Test 1: MRS_SCA dummy build\n");
    MRS::Algorithms::SCA synergy_creation_algorithm;
    //Create dummy robot - text printer
    MRS::Task::DummyRobot robot;
    synergy_creation_algorithm.SetRobot(&robot);
    //Create binary state predictor
    MRS::Algorithms::BinaryStateRandomPredictor coin_predictor;
    synergy_creation_algorithm.SetPredictor(&coin_predictor);
    //Create an arithmetic calculator
    MRS::Algorithms::ArithmeticCalculator calc;
    synergy_creation_algorithm.SetCalculator(&calc);
    //synergy_creation_algorithm.ExecuteBestBehaviour();
    printf("Test 1: Done\n");
    return 0;
}