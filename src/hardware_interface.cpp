#include <cmath>
#include <iostream>
#include <vector>

#define PROTOCOL_VERSION 2.0
#define pos2rad(x) (x/2048.0 - 1.0)*M_PI
#define rad2pos(x) round((x / M_PI + 1.0) * 2048.0)
#define vel2rps(x) x

#define PRESENT_POSITION_SIZE 4 //in bytes
#define PRESENT_VELOCITY_SIZE 4 //in bytes
#define GOAL_POSITION_SIZE 4 //in bytes
#define GOAL_VELOCITY_SIZE 4 //in bytes

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include "dynamixel_sdk/dynamixel_sdk.h"

#include "../include/config_data.h"


class myDynamixelRobot: public hardware_interface::RobotHW
{
    // State vectors for all joints. 
    std::vector<double> cmd,pos,vel,eff;

    // vectors for writing register values. 4bytes per element of vector
    std::vector<std::array<uint8_t, 4>> goalPositions, goalVelocities; 

    // declare the jointState, positionJoint and velocityJoint interfaces
    hardware_interface::JointStateInterface joint_state_inf;
    hardware_interface::PositionJointInterface joint_position_inf;
    hardware_interface::VelocityJointInterface joint_velocity_inf;

    // a vector for storing the configuration data and register addresses for all servos
    servoInfoList joints_list;

    // dynamixel objects for handling port and transmission
    dynamixel::PortHandler *port;
    dynamixel::PacketHandler *packet;

    // dynamixel objects for groupWrite and groupRead
    dynamixel::GroupBulkRead *bulkReader;
    dynamixel::GroupBulkWrite *bulkWriter;

    public:
    myDynamixelRobot(){

        // read data from the yaml files. REPLACE WITH PARAM
        servoTypesMap servo_types = readServoTypes("/home/arthur/dynamixel_ws/src/dynamixel_hardware_interface/config/servo_types.yaml");
        joints_list = readServoInfo("/home/arthur/dynamixel_ws/src/dynamixel_hardware_interface/config/servo_info.yaml", servo_types);

        // populate the state vectors
        for(int i =0; i < joints_list.size(); i++){
            cmd.push_back(0.0);
            pos.push_back(0.0);
            vel.push_back(0.0);
            eff.push_back(0.0);
            goalPositions.push_back({0,0,0,0});
            goalVelocities.push_back({0,0,0,0});
        }

        // init joint state interface.
        for(int i=0; i<joints_list.size(); i++){

            // one handle per joint. Handle has references needs the joint Name, pos, vel and eff
            hardware_interface::JointStateHandle handle(joints_list[i].name, &pos[i], &vel[i], &eff[i]);

            // register the handle with the interface
            joint_state_inf.registerHandle(handle);
        }
        // finally register the interface itself
        registerInterface(&joint_state_inf);

        // init joint position and velocity interfaces
        for(int i=0; i<joints_list.size(); i++){

            // position and velocity interfaces need handles of type JointHandle
            hardware_interface::JointHandle handle(joint_state_inf.getHandle(joints_list[i].name), &cmd[i]);
            
            if(joints_list[i].joint_type == std::string("position")){
                joint_position_inf.registerHandle(handle);
            }
            else if(joints_list[i].joint_type == std::string("velocity")){
                joint_velocity_inf.registerHandle(handle);
            }
            else{
                ROS_ERROR_STREAM("unsupported joint_type " << joints_list[i].joint_type);
            }
        }
        registerInterface(&joint_position_inf);
        registerInterface(&joint_velocity_inf);

        // init the hardware

        const char u2d2_port[] = "/dev/ttyUSB0";

        if(!initHW(u2d2_port, 1000000)){
            ROS_ERROR_STREAM("hardware initialization FAILED for port: "<< u2d2_port);
            exit(1);
        }

    }
    ~myDynamixelRobot()
    {
        //close the port
        port->closePort();
        ROS_INFO_STREAM("port closed. Shutting down HW Interface");

        delete bulkReader,bulkWriter, port, packet;
    }

    bool initHW(const char* device_name, const int baud)
    {
        int dxl_comm_result = COMM_TX_FAIL;

        // open the port
        port = dynamixel::PortHandler::getPortHandler(device_name);
        packet = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        bulkReader = new dynamixel::GroupBulkRead(port, packet);
        bulkWriter = new dynamixel::GroupBulkWrite(port, packet);

        if (!port->openPort()) {
            ROS_ERROR("Failed to open the port!");
            return false;
        }

        if (!port->setBaudRate(baud)) {
            ROS_ERROR("Failed to set the baudrate!");
            return false;
        }

        ROS_INFO("writing data to %ld servos", joints_list.size());

        // write all the configuration data to the servos like PID values and operating mode.
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
            
            // set the PID values
            for(int i=0; i<5; i++)
                    dxl_comm_result = packet->write2ByteTxRx(port, servo.id, values[i][0], (uint16_t)values[i][1]);

            // renable torque
            dxl_comm_result = packet->write1ByteTxRx(port, servo.id, servo.torque_enable, 1);

            // check for any write failures
            if (dxl_comm_result != COMM_SUCCESS) {
                ROS_ERROR("Failed to set Dynamixel ID %d", servo.id);
                return false;
            }
            ROS_INFO("Servo ID: %d connected and set up",servo.id);
        }

        // prep the bulk reader and writer with params
        for(int i =0; i<joints_list.size(); i++){
            bool result = true;
            // reader
            result &= bulkReader->addParam(joints_list[i].id, joints_list[i].present_position, PRESENT_POSITION_SIZE);
            result &= bulkReader->addParam(joints_list[i].id, joints_list[i].present_velocity, PRESENT_VELOCITY_SIZE);

            // writer
            result &= bulkWriter->addParam(joints_list[i].id, joints_list[i].goal_position, GOAL_POSITION_SIZE, goalPositions[i].data());
            result &= bulkWriter->addParam(joints_list[i].id, joints_list[i].goal_velocity, GOAL_VELOCITY_SIZE, goalVelocities[i].data());
        }   
        return true;
    }

    bool read()
    {
        // call the bulk reader
        if(bulkReader->txRxPacket() == COMM_SUCCESS){
            
            for(int i=0; i<joints_list.size(); i++){
                uint32_t bytesPos = bulkReader->getData(joints_list[i].id, joints_list[i].present_position, PRESENT_POSITION_SIZE);
                uint32_t bytesVel = bulkReader->getData(joints_list[i].id, joints_list[i].present_velocity, PRESENT_VELOCITY_SIZE);
                this->pos[i] = pos2rad(bytesPos);
                this->vel[i] = vel2rps(bytesVel);
            }
            return true;
        }
        else{
            return false;
        }

        return true;
    }

    bool write()
    {
        //convert cmd to either position/velocity bytes
        for(int i=0; i<joints_list.size(); i++){

            if(joints_list[i].joint_type == std::string("position")){
                // update the goalPosition vector with value from cmd;
            }
            else if(joints_list[i].joint_type == std::string("velocity")){
                //update the goalVelocities vector with value from cmd
            }
        }
        return true;
    }

    // takes radians and writes the equivalent position to a uint8[4] array (LSB to MSB).
    void rad2bytes(float rads, uint8_t *bytes){
        uint32_t pos = rad2pos(rads);
        for(int i=0; i<4; i++){
            bytes[i] = (pos  >> 8*i) & 0xFF;
        } 
    }
};

int main()
{
    myDynamixelRobot obj = myDynamixelRobot();
    obj.read();
    return 0;
}