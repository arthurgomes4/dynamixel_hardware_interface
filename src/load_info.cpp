#include "../include/config_data.h"

int main()
{
    auto servo_types = readServoTypes("../config/servo_types.yaml");
    auto joints = readServoInfo("../config/servo_info.yaml", servo_types);
    
    return 0;
}