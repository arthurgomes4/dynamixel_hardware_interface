# pragma once 

#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <yaml-cpp/yaml.h>

extern const char* dynamixelAddrNames[];

struct dynamixelServo
{
    std::string name;

    int id;
    int operating_mode;
    int position_pid[3];
    int velocity_pi[2];

    uint8_t operating_mode_reg;
    uint8_t torque_enable;
    uint8_t goal_position;
    uint8_t goal_velocity;
    uint8_t present_load;
    uint8_t present_velocity;
    uint8_t present_position;
    uint8_t position_P;
    uint8_t position_I;
    uint8_t position_D;
    uint8_t velocity_P;
    uint8_t velocity_I;
};

std::map<std::string, std::map< std::string, uint8_t>> readServoTypes(const char* file_name);

std::vector<dynamixelServo> readServoInfo(const char* file_name, std::map<std::string, std::map< std::string, uint8_t>> servo_types);