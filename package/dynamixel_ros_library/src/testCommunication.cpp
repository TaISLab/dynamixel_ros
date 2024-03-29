#include <dynamixel_ros_library.h>

dynamixelMotor J1("J1",1);


int main(int argc, char *argv[])
{
    char* port_name;
    int baud_rate;
    float protocol_version;

    if (argc != 4)
    {
        printf("Please set '-port_name', '-protocol_version' '-baud_rate' arguments for connected Dynamixels\n");
        return 0;
    } else
    {
        port_name = argv[1];
        protocol_version = atoi(argv[2]);
        baud_rate = atoi(argv[3]);
    }

    dynamixelMotor::iniComm(port_name,protocol_version,baud_rate);
    J1.setControlTable();

    J1.getBaudrate();
    J1.setReturnDelayTime(500);
    int time = J1.getReturnDelayTime();
    
    return 1;
}
