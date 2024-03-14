#include <dynamixel_ros_library.h>

dynamixelMotor J1("J1","MX430-W210-T",1);

int main(int argc, char *argv[])
{
    char* port_name = "/dev/ttyUSB0";
    int baud_rate = 57600;
    float protocol_version = 2.0;

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

    return 1;
}
