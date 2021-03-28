#include <cstdio>
//#include <boost/asio.hpp>
#include <MRS/Include/Device.hpp>
#include "FastSLAM.h"
#include "SpatialObjectSensor_File.h"
#include "DistanceSensor_Driver.h"
#include "ObjectSensor_Driver.h"
#include <MRS/Include/View2DPosition.hpp>
#include <list>
#include <fstream>
#include <iostream>

static void UpdatePosition(MRS::Device::View2DPosition*, std::string * data);
static void UpdatePosition(MRS::Device::View2DPosition*, double, double, double);
static double ParseForX(std::string *);
static double ParseForY(std::string *);
static double ParseForA(std::string *);



int main()
{
    printf("hello from FastSlamTest!\n");
    
    std::string message, sensorString;
    std::list<std::string> sensorStrings;
    int i = 0;
    std::fstream myfile;
    std::string INPUT_FILE_NAME = "SensorData1.dat";
    std::string INPUT_FILE_PATH = "//home//research//projects//FastSlamTest//";
    std::string INPUT_FILE = INPUT_FILE_PATH + INPUT_FILE_NAME;
    myfile.open(INPUT_FILE, std::ios::in);
    if (myfile.is_open()) {
        while (std::getline(myfile, sensorString)) {
            if (myfile.eof())break;
            //taskTimeline[i] = SECONDS_TO_STEPS * stoi(taskString.substr(0, taskString.find_first_of(' ')));
            sensorStrings.emplace_back(sensorString);
            std::cout << sensorStrings.back() << "\n";
            i++;
        }
    }
    myfile.close();

    MRS::FastSLAM slam;
    MRS::SpatialObjectSensor_File file_sensor;
    MRS::DistanceSensor_Driver file_distance_sensor;
    MRS::ObjectSensor_Driver file_object_sensor;
    MRS::Device::View2DPosition position_view;
    file_sensor.SetDistanceSensor((MRS::Device::DistanceSensor*)&file_distance_sensor);
    file_sensor.SetObjectSensor((MRS::Device::ObjectSensor*)&file_object_sensor);
    // Init sensors
    file_sensor.Init();
    // create world
    // create FastSLAM
    
    std::string path_string = "";

   double tick = 0.2;
    //double total = 0;
   // std::string movement = "(move (1 2 3))";
    //double finish = 1;
    std::string temp = "(see 0.5 (10 10 23 5.5 9.3))"; //change to string updater later
    file_sensor.SetData(&temp);
    file_distance_sensor.SetSensorData(&temp);
    file_object_sensor.SetString(&temp);
    for (std::string s : sensorStrings) {
        if (s[1] == 's') {
            file_sensor.SetData(&s);
            file_sensor.Update((long)(tick * 1000000));
            slam.Update(tick, file_distance_sensor.GetView());
        }
        else if (s[1] == 'm') {
            UpdatePosition(&position_view, &s);
            slam.Update(tick, &position_view);
            //slam.OutputMap();
        }

        path_string = path_string + slam.OutputMapString();
        //std::cout << "Sensor input: " << path_string << "\n";
        //slam.OutputMap();
        std::cout << "Sensor input: " << s << "\n";
        //std::cout << "Test output " << tick << "\n";
    }
    
    std::fstream myfile3;
    std::string OUTPUT_FILE_NAME2 = "EntireParticlePath.res";
    std::string OUTPUT_FILE_PATH2 = "/home/research/projects/FastSlamTest/";
    std::string OUTPUT_FILE2 = OUTPUT_FILE_PATH2 + OUTPUT_FILE_NAME2;
    myfile3.open(OUTPUT_FILE2, std::ios::out);
    if (myfile3.is_open()) {
        myfile3 << path_string;
    }
    myfile3.close();
    slam.OutputMap();

    std::fstream myfile2;
    std::string OUTPUT_FILE_NAME = "ParticleResult.res";
    std::string OUTPUT_FILE_PATH = "/home/research/projects/FastSlamTest/";
    std::string OUTPUT_FILE = OUTPUT_FILE_PATH + OUTPUT_FILE_NAME;
    myfile2.open(OUTPUT_FILE, std::ios::out);
    if (myfile2.is_open()) {
        myfile2 << slam.OutputMapString();
    }
    myfile2.close();
    
    file_sensor.DeInit();
    //Kartes ģenerēšana
}

static void UpdatePosition(MRS::Device::View2DPosition* view, std::string * data) {
    // change x, y and angle coords according to file
    UpdatePosition(view, ParseForX(data), ParseForY(data), ParseForA(data));
}

static void UpdatePosition(MRS::Device::View2DPosition* view, double x, double y, double a) {
    // change x, y and angle coords according to file
    view->SetX(x);
    view->SetY(y);
    view->SetA(a);
}

static double ParseForX(std::string* data) {
    // Format
    // (move (X Y A))
    double x = 0.0;
    int currentParentesiesOpen = 0, next = 0, firstTemp, secondTemp;
    std::string temp, dataSubstring, xSubstring;
    // copy
    temp = "" + *data;
    // get substring
    currentParentesiesOpen = (int)temp.find_first_of('(', 0);
    next = (int)temp.find_first_of('(', currentParentesiesOpen + 1);
    currentParentesiesOpen = next;
    next = (int)temp.find_first_of(')', currentParentesiesOpen + 1);
    //printf("Data substring info: %d %c %d %c\n", currentParentesiesOpen, temp[currentParentesiesOpen], next, temp[next + 1]);
    dataSubstring = "" + temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
    //printf("Data substring: %s\n", dataSubstring.c_str());
    firstTemp = 0;
    secondTemp = (int)dataSubstring.find_first_of(' ', firstTemp + 1);
    //printf("Data substring info: %d %c %d %c\n", firstTemp, dataSubstring[firstTemp], secondTemp, dataSubstring[secondTemp+1]);
    xSubstring = "" + dataSubstring.substr(firstTemp, secondTemp - firstTemp);
    //printf("X substring: %s\n", xSubstring.c_str());
    // parse double
    try {
        x = std::stod(xSubstring);
    }
    catch (...) {
        x = -4.43;
        printf(" Sub: %s \n", xSubstring.c_str());
        printf(" Num error \n");
    }
    // normalize
    //x = angle * PI / 180.0;
    return x;
}

static double ParseForY(std::string* data) {
    double y = 0.0;
    int currentParentesiesOpen = 0, next = 0, firstTemp, secondTemp;
    std::string temp, dataSubstring, ySubstring;
    // copy
    temp = "" + *data;
    // get substring
    currentParentesiesOpen = (int)temp.find_first_of('(', 0);
    next = (int)temp.find_first_of('(', currentParentesiesOpen + 1);
    currentParentesiesOpen = next;
    next = (int)temp.find_first_of(')', currentParentesiesOpen + 1);
    dataSubstring = "" + temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
    firstTemp = (int)dataSubstring.find_first_of(' ', 0);
    secondTemp = (int)dataSubstring.find_first_of(' ', firstTemp + 1);
    ySubstring = "" + dataSubstring.substr(firstTemp , secondTemp - firstTemp);
    //printf("y substring: %s\n", ySubstring.c_str());
    // parse double
    try {
        y = std::stod(ySubstring);
    }
    catch (...) {
        y = -4.43;
        printf(" Sub: %s \n", ySubstring.c_str());
        printf(" Num error \n");
    }
    // normalize
    //angle = angle * PI / 180.0;
    return y;
}

static double ParseForA(std::string* data) {
    double angle = 0.0;
    int currentParentesiesOpen = 0, next = 0, firstTemp, secondTemp;
    std::string temp, dataSubstring, angleSubstring;
    // copy
    temp = "" + *data;
    // get substring
    currentParentesiesOpen = (int)temp.find_first_of('(', 0);
    next = (int)temp.find_first_of('(', currentParentesiesOpen + 1);
    currentParentesiesOpen = next;
    next = (int)temp.find_first_of(')', currentParentesiesOpen + 1);
    dataSubstring = "" + temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
    firstTemp = (int)dataSubstring.find_first_of(' ', 0);
    secondTemp = (int)dataSubstring.find_first_of(' ', firstTemp + 1);
    firstTemp = secondTemp;
    secondTemp = (int)dataSubstring.find_first_of(')', firstTemp + 1);
    angleSubstring = "" + dataSubstring.substr(firstTemp, secondTemp - firstTemp);
    //printf("Angle substring: %s\n", angleSubstring.c_str());
    // parse double
    try {
        angle = std::stod(angleSubstring);
    }
    catch (...) {
        angle = -4.43;
        printf(" Sub: %s \n", angleSubstring.c_str());
        printf(" Num error \n");
    }
    // normalize
    angle = angle * PI / 180.0;
    return angle;
}