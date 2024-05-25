#include "ros/ros.h"
#include "std_msgs/Int32.h" 
#include <dynamixel_ros_library.h>

dynamixelMotor myDynamixel;

void userInputCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int userValue = msg->data;

    if(myDynamixel.getOperatingMode() != "POSITION_CONTROL_MODE")
    {
        myDynamixel.setTorqueState(false);
        myDynamixel.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
        myDynamixel.setTorqueState(true);
    }

    myDynamixel.setGoalPosition(userValue);
}

int main(int argc, char **argv)
{
    char* port_name;
    int baud_rate, dmxl_id;
    float protocol_version;

    if (argc != 5)
    {
        printf("Please set '-port_name', '-protocol_version' '-baud_rate' '-dynamixel_id' arguments for connected Dynamixels\n");
        return 0;
    } else
    {
        port_name = argv[1];
        protocol_version = atoi(argv[2]);
        baud_rate = atoi(argv[3]);
        dmxl_id = atoi(argv[4]);
    }

    myDynamixel = dynamixelMotor("J1",dmxl_id);
    dynamixelMotor::iniComm(port_name,protocol_version,baud_rate);    
    myDynamixel.setControlTable();


    ros::init(argc, argv, "goal_position_reader");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("user_input", 10, userInputCallback);

    ros::spin();

    return 0;
}
