#include "../include/config_data.h"
using namespace std;

const char* dynamixelAddrNames[12] = {
    "operating_mode",
    "torque_enable",
    "goal_position",
    "goal_velocity",
    "present_load",
    "present_velocity",
    "present_position",
    "position_P",
    "position_I",
    "position_D",
    "velocity_P",
    "velocity_I"
};

map<string, map< string, uint8_t>> readServoTypes(const char* file_name)
{
    YAML::Node servo_types_yaml = YAML::LoadFile(file_name);
    if(!servo_types_yaml.size())
        cerr << file_name << " is empty";

    map<string, map< string, uint8_t>> servo_types;

    for(auto type : servo_types_yaml)
    {   
        string servo_name = type.first.as< string>();
        map< string, uint8_t> addressTable;

        for(const char* name : dynamixelAddrNames){
            addressTable[name] = (uint8_t)type.second[name].as<int>();
        }
        servo_types[servo_name] = addressTable;
    }
    return servo_types;
}

vector<dynamixelServo> readServoInfo(const char* file_name, map<string, map< string, uint8_t>> servo_types)
{
    YAML::Node servo_info_yaml = YAML::LoadFile(file_name);
    if(!servo_info_yaml)
        cerr << file_name << " is empty";
    
    vector<dynamixelServo> joints;

    for(auto item : servo_info_yaml)
    {
        dynamixelServo joint;
        string type = item.second["type"].as<string>();

        joint.name = item.first.as<string>();
        joint.joint_type = item.second["joint_type"].as<string>();
        joint.id = item.second["id"].as<int>();
        joint.operating_mode = item.second["operating_mode"].as<int>();
      
        joint.position_pid[0] = item.second["position_pid"]["p"].as<int>();
        joint.position_pid[1] = item.second["position_pid"]["i"].as<int>();
        joint.position_pid[2] = item.second["position_pid"]["d"].as<int>();

        joint.velocity_pi[0] = item.second["velocity_pi"]["p"].as<int>();
        joint.velocity_pi[1] = item.second["velocity_pi"]["i"].as<int>();

        joint.operating_mode_reg = servo_types[type]["operating_mode"];
        joint.torque_enable = servo_types[type]["torque_enable"];
        joint.goal_position = servo_types[type]["goal_position"];
        joint.goal_velocity = servo_types[type]["goal_velocity"];
        joint.present_load = servo_types[type]["present_load"];
        joint.present_velocity = servo_types[type]["present_velocity"];
        joint.present_position = servo_types[type]["present_position"];
        joint.position_P = servo_types[type]["position_P"];
        joint.position_I = servo_types[type]["position_I"];
        joint.position_D = servo_types[type]["position_D"];
        joint.velocity_P = servo_types[type]["velocity_P"];
        joint.velocity_I= servo_types[type]["velocity_I"];

        joints.push_back(joint);
    }
    return joints;
}