#include <ros/ros.h>
#include <dynamixel_ros_library.h>

void switchLed(dynamixelMotor& motor)
{
   motor.setLedState(!motor.getLedState());
}

int main(int argc, char *argv[])
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

    dynamixelMotor J1("J1",dmxl_id);
    dynamixelMotor::iniComm(port_name,protocol_version,baud_rate);
    
    J1.setControlTable();

    // ROS node init
    ros::init(argc, argv, "blinkingLED");
    ros::NodeHandle nh;

    // Callback creation
    auto callbackFunction = std::bind(switchLed,J1);
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), callbackFunction);

    ros::spin();

    return 0;
}
