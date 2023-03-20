#include "../include/config_data.h"
#include "ros/ros.h"

void printTypesMap(servoTypesMap map)
{
    // iterate over the outer map
    for (const auto& outerPair : map){
        std::cout << "Type: " << outerPair.first << std::endl;

        // iterate over the inner map
        for (const auto& innerPair : outerPair.second){
            std::cout << "\t" << innerPair.first << ": " << static_cast<int>(innerPair.second) << std::endl;
        }
    }
}

int main()
{
    servoTypesMap temp;
    std::cout << sizeof(temp) << "\n";
    auto servo_types = readServoTypes("/home/arthur/dynamixel_ws/src/dynamixel_hardware_interface/config/servo_types.yaml");
    auto joints = readServoInfo("/home/arthur/dynamixel_ws/src/dynamixel_hardware_interface/config/servo_info.yaml", servo_types);
    temp = servo_types;
    std::cout << &servo_types << " " << &temp << "\n";
    std::cout << sizeof(temp) << "\n";

    printTypesMap(temp);
    std::cout << joints.size() << "\n";
    return 0;
}