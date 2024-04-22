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
    J1.setReturnDelayTime(2);
    int time = J1.getReturnDelayTime();
    J1.configDriveMode(false,false,false,false);
    
    std::printf(J1.getOperatingMode().c_str());
    std::printf("\n");

    J1.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
    J1.getShadowID();
    J1.setShadowID(253);

    J1.getProcotolType();
    J1.getHomingOffset();
    J1.setHomingOffset(360);
    J1.getHomingOffset();

    J1.getMovingThreshold();
    J1.setMovingThreshold(0);
    J1.getMovingThreshold();

    J1.getTempLimit();
    J1.setTempLimit(78);
    J1.getTempLimit();

    J1.getMaxVoltageLimit();
    J1.setMaxVoltageLimit(16);
    J1.getMaxVoltageLimit();

    J1.getMinVoltageLimit();
    J1.setMinVoltageLimit(9.5);
    J1.getMinVoltageLimit();

    J1.getPWMLimit();
    J1.setPWMLimit(100);
    J1.getPWMLimit();

    J1.getCurrentLimit();
    J1.setCurrentLimit(3200);
    J1.getCurrentLimit();

    J1.getVelLimit();
    J1.setVelLimit(230);
    J1.getVelLimit();

    J1.getMaxPosLimit();
    J1.getMinPosLimit();

    J1.configStartup(true,true);

    J1.configShutdown(false,true,false,true,true);

    J1.getTorqueState();

    return 1;
}
