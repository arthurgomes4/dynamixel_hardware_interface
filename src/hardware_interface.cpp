#include <cmath>
#include <iostream>
#include <vector>

#define PROTOCOL_VERSION 2.0
#define pos2rad(x) (x/2048.0 - 1.0)*M_PI
#define rad2pos(x) round((x / M_PI + 1.0) * 2048.0)

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include "dynamixel_sdk/dynamixel_sdk.h"

#include "../include/config_data.h"


class myDynamixelRobot: public hardware_interface::RobotHW
{
    std::vector<double> cmd,pos,vel,eff;

    hardware_interface::JointStateInterface joint_state_inf;
    hardware_interface::PositionJointInterface joint_position_inf;
    hardware_interface::VelocityJointInterface joint_velocity_inf;

    servoInfoList joints_list;
    dynamixel::PortHandler *port;
    dynamixel::PacketHandler *packet;

    public:
    myDynamixelRobot(){

        // read data
        servoTypesMap servo_types = readServoTypes("/home/arthur/dynamixel_ws/src/dynamixel_hardware_interface/config/servo_types.yaml");
        joints_list = readServoInfo("/home/arthur/dynamixel_ws/src/dynamixel_hardware_interface/config/servo_info.yaml", servo_types);

        // create state vectors
        for(int i =0; i < joints_list.size(); i++){
            cmd.push_back(0.0);
            pos.push_back(0.0);
            vel.push_back(0.0);
            eff.push_back(0.0);
        }

        // init joint state interface 
        for(int i=0; i<joints_list.size(); i++){
            hardware_interface::JointStateHandle handle(joints_list[i].name, &pos[i], &vel[i], &eff[i]);
            joint_state_inf.registerHandle(handle);
        }
        registerInterface(&joint_state_inf);

        // init joint position and velocity interfaces
        for(int i=0; i<joints_list.size(); i++){
            hardware_interface::JointHandle handle(joint_state_inf.getHandle(joints_list[i].name), &cmd[i]);
            
            if(joints_list[i].joint_type == std::string("position"))
            {
                joint_position_inf.registerHandle(handle);
            }
            else if(joints_list[i].joint_type == std::string("velocity"))
            {
                joint_velocity_inf.registerHandle(handle);
            }
            else
            {
                ROS_ERROR_STREAM("unsupported joint_type" << joints_list[i].joint_type);
            }
        }
        registerInterface(&joint_position_inf);
        registerInterface(&joint_velocity_inf);


        // init the hardware
        if(!initHW("/dev/ttyUSB0", 1000000))
            exit(1);

    }
    ~myDynamixelRobot()
    {
        //close the port
        port->closePort();
        ROS_INFO_STREAM("port: closed. Shutting down HW Interface");
    }

    bool initHW(const char* device_name, const int baud)
    {
        int dxl_comm_result = COMM_TX_FAIL;

        // open the port
        port = dynamixel::PortHandler::getPortHandler(device_name);
        packet = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        if (!port->openPort()) {
            ROS_ERROR("Failed to open the port!");
            return false;
        }

        if (!port->setBaudRate(baud)) {
            ROS_ERROR("Failed to set the baudrate!");
            return false;
        }

        ROS_INFO("writing data to %ld servos", joints_list.size());

        for(const dynamixelServo& servo : joints_list)
        {
            // shut down torque to make changes to registers
            dxl_comm_result = packet->write1ByteTxRx(port, servo.id, servo.torque_enable, 0);
            
            // set operating mode
            dxl_comm_result = packet->write1ByteTxRx(port, servo.id, servo.operating_mode_reg, servo.operating_mode);
            
            int values[5][2] = {{servo.position_P, servo.position_pid[0]},
                                {servo.position_I, servo.position_pid[1]},
                                {servo.position_D, servo.position_pid[2]},
                                {servo.velocity_P, servo.velocity_pi[0]},
                                {servo.velocity_I, servo.velocity_pi[1]},};
            
            for(int i=0; i<5; i++)
                    dxl_comm_result = packet->write2ByteTxRx(port, servo.id, values[i][0], (uint16_t)values[i][1]);

            // renable torque
            dxl_comm_result = packet->write1ByteTxRx(port, servo.id, servo.torque_enable, 1);
            if (dxl_comm_result != COMM_SUCCESS) {
                ROS_ERROR("Failed to set Dynamixel ID %d", servo.id);
                return false;
            }
            ROS_INFO("Servo ID: %d connected and set up",servo.id);
        }

        return true;
    }

    bool read()
    {
        return true;
    }

    bool write()
    {
        return true;
    }

};

int main()
{
    myDynamixelRobot obj = myDynamixelRobot();
    return 0;
}