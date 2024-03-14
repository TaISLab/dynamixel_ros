#include <dynamixel_ros_library.h>
#include <ros/ros.h>

dynamixel::PortHandler *myPortHandler;
dynamixel::PacketHandler *myPacketHandler;

dynamixelMotor::dynamixelMotor(std::string IDENTIFICATOR,std::string MODEL, int ID)
{
    this->IDENTIFICATOR = IDENTIFICATOR;
    this->MODEL = MODEL;
    this->ID = ID;
}

dynamixelMotor::~dynamixelMotor()
{
    
}

bool dynamixelMotor::iniComm(char* PORT_NAME, float PROTOCOL_VERSION, int BAUDRATE)
{
    myPortHandler = dynamixel::PortHandler::getPortHandler(PORT_NAME);
    myPacketHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!myPortHandler->openPort()) 
    {
        ROS_ERROR("Failed to open the port: %s !", PORT_NAME);
        return false;
    } else if (!myPortHandler->setBaudRate(BAUDRATE))  
    {
        ROS_ERROR("Failed to set the baudrate: %d !", BAUDRATE);
        return false;
    } else
    {
        ROS_INFO("SUCCESS");
        return true;
    }

}

int dynamixelMotor::getMotorID()
{

}
