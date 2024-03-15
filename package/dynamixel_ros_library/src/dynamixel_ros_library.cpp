#include <dynamixel_ros_library.h>
#include <ros/ros.h>

dynamixel::PortHandler *myPortHandler;
dynamixel::PacketHandler *myPacketHandler;


dynamixelMotor::dynamixelMotor(std::string IDENTIFICATOR, int ID)
{
    this->IDENTIFICATOR = IDENTIFICATOR;
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
        ROS_INFO("Initialization success");
        return true;
    }
}

void dynamixelMotor::setControlTable()
{
    // Definition of necessary variables
    uint16_t *model_number = new uint16_t[1];
    uint8_t dxl_error;

    // Checking the DMXL model (adress 0 in all DMXLS)
    int dxl_comm_result = myPacketHandler->read2ByteTxRx(myPortHandler, this->ID, 0, model_number, &dxl_error);

    if(dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("Error reading the model number of DMXL: %d",this->ID);
        ROS_ERROR("Error code: %d", dxl_error);
    }

    switch(static_cast<int>(*model_number))
    {
        // XW SERIES
        case 1180:
            this->MODEL = "XW540-T140";
        break;

        case 1170:
            this->MODEL = "XW540-T260";
        break;

        case 1280:
            this->MODEL = "XW430-T200";
        break;

        case 1270:
            this->MODEL = "XW430-T333";
        break;

        // XD SERIES
        case 1111:
            this->MODEL = "XD540-T150";
        break;

        case 1101:
            this->MODEL = "XD540-T270";
        break;

        case 1011:
            this->MODEL = "XD430-T210";
        break;

        case 1001:
            this->MODEL = "XD430-T350";
        break;

        // XH SERIES
        case 1110:
            this->MODEL = "XH540-W150";
        break;

        case 1100:
            this->MODEL = "XH540-W270";
        break;

        case 1150:
            this->MODEL = "XH540-V150";
        break;

        case 1140:
            this->MODEL = "XH540-V270";
        break;

        case 1010:
            this->MODEL = "XH430-W210";
        break;

        case 1000:
            this->MODEL = "XH430-W350";
        break;

        case 1050:
            this->MODEL = "XH430-V210";
        break;

        case 1040:
            this->MODEL = "XH430-V350";
        break;
 
        // XM SERIES
        case 1130:
            this->MODEL = "XM540-W150";
        break;

        case 1120:
            this->MODEL = "XM540-W270";
        break;

        case 1030:
            this->MODEL = "XM430-W210";
        break;

        case 1020:
            this->MODEL = "XM430-W350";
        break;

        // XC SERIES
        case 1160:
            this->MODEL = "2XC430-W250";
        break; 

        case 1070:
            this->MODEL = "XC430-W150";
        break;

        case 1080:
            this->MODEL = "XC430-W240";
        break;

        case 1220:
            this->MODEL = "XC330-T288";
        break;

        case 1210:
            this->MODEL = "XC330-T181";
        break;

        case 1230:
            this->MODEL = "XC330-M181";
        break;

        case 1240:
            this->MODEL = "XC330-M288";
        break;

        // XL SERIES
        case 1060:
            this->MODEL = "XL430-W250";
        break;

        case 1190:
            this->MODEL = "XL330-M077";
        break;

        case 1200:
            this->MODEL = "XL330-M288";
        break;

        case 350:
            this->MODEL = "XL320";
        break;

        default:
            this->MODEL = "No model";
        break;

    }
    
    ROS_INFO("Control table set for: %s",this->MODEL.c_str());
}

int dynamixelMotor::getID()
{
    return this->ID;
}

void dynamixelMotor::setID()
{

}

std::string dynamixelMotor::getModel()
{
    return this->MODEL;
}

int dynamixelMotor::getBaudrate()
{
    uint8_t dxl_error = 0;
    uint8_t *data = new uint8_t[1];
    int baudrate;

    int dxl_comm_result = myPacketHandler->read1ByteTxRx(myPortHandler, this->ID, 8, data, &dxl_error);
        
    if(dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to read from the LED for Dynamixel ID %d", this->ID);
        ROS_ERROR("Error code: %d", dxl_error);
        return 0;
    }

    switch(static_cast<int>(*data))
    {
        case 0:

        break;

        case 1:
            baudrate = 57600;
        break;

        case 2:

        break;

        case 3:

        break;

        default:

        break;

    }

    ROS_INFO("Getting baudrate for Dynamixel ID: %d",this->ID);
    ROS_INFO("Current baudrate is: %d bps",baudrate);
    return baudrate;
}
