#include <dynamixel_ros_library.h>
#include <ros/ros.h>

dynamixel::PortHandler *myPortHandler = nullptr;
dynamixel::PacketHandler *myPacketHandler = nullptr;

std::map<int, std::string> dynamixelMotor::DMXL_MODELS = {
    // XW SERIES
    {1180, "XW540-T140"},
    {1170, "XW540-T260"},
    {1280, "XW430-T200"},
    {1270, "XW430-T333"},

    // XD SERIES
    {1111, "XD540-T150"},
    {1101, "XD540-T270"},
    {1011, "XD430-T210"},
    {1001, "XD430-T350"},

    // XH SERIES
    {1110, "XH540-W150"},
    {1100, "XH540-W270"},
    {1150, "XH540-V150"},
    {1140, "XH540-V270"},
    {1010, "XH430-W210"},
    {1000, "XH430-W350"},
    {1050, "XH430-V210"},
    {1040, "XH430-V350"},

    // XM SERIES
    {1130, "XM540-W150"},
    {1120, "XM540-W270"},
    {1030, "XM430-W210"},
    {1020, "XM430-W350"},

    // XC SERIES
    {1160, "2XC430-W250"},
    {1070, "XC430-W150"},
    {1080, "XC430-W240"},
    {1220, "XC330-T288"},
    {1210, "XC330-T181"},
    {1230, "XC330-M181"},
    {1240, "XC330-M288"},

    // XL SERIES
    {1060, "XL430-W250"},
    {1190, "XL330-M077"},
    {1200, "XL330-M288"},
    {350, "XL320"}
};

// Compatibility: XW, XD430, XH430, XM430,
std::map<std::string, int> dynamixelMotor::ADDR_DMXL22 = {
    // EEPROM AREA
    {"MODEL_NUMBER", 0},
    {"MODEL_INFORMATION", 2},
    {"FIRMWARE_VERSION", 6},
    {"ID", 7},
    {"BAUDRATE", 8},
    {"RETURN_DELAY_TIME", 9},
    {"DRIVE_MODE", 10},
    {"OPERATING_MODE", 11},
    {"SECONDARY_ID", 12},
    {"PROTOCOL_TYPE", 13},
    {"HOMING_OFFSET", 20},
    {"MOVING_THRESHOLD", 24},
    {"TEMPERATURE_LIMIT", 31},
    {"MAX_VOLTAGE_LIMIT", 32},
    {"MIN_VOLTAGE_LIMIT", 34},
    {"PWM_LIMIT", 36},
    {"CURRENT_LIMIT", 38},
    {"VELOCITY_LIMIT", 44},
    {"MAX_POSITION_LIMIT", 48},
    {"MIN_POSITION_LIMIT", 52},
    {"STARTUP_CONFIGURATION", 60},
    {"SHUTDOWN", 63}
};

// Compatibility: XD540, XH540, XM540, 
std::map<std::string, int> dynamixelMotor::ADDR_DMXL25 = {
    // EEPROM AREA
    {"MODEL_NUMBER", 0},
    {"MODEL_INFORMATION", 2},
    {"FIRMWARE_VERSION", 6},
    {"ID", 7},
    {"BAUDRATE", 8},
    {"RETURN_DELAY_TIME", 9},
    {"DRIVE_MODE", 10},
    {"OPERATING_MODE", 11},
    {"SECONDARY_ID", 12},
    {"PROTOCOL_TYPE", 13},
    {"HOMING_OFFSET", 20},
    {"MOVING_THRESHOLD", 24},
    {"TEMPERATURE_LIMIT", 31},
    {"MAX_VOLTAGE_LIMIT", 32},
    {"MIN_VOLTAGE_LIMIT", 34},
    {"PWM_LIMIT", 36},
    {"CURRENT_LIMIT", 38},
    {"VELOCITY_LIMIT", 44},
    {"MAX_POSITION_LIMIT", 48},
    {"MIN_POSITION_LIMIT", 52},
    {"EXTERNAL_PORT_MODE_1", 56},
    {"EXTERNAL_PORT_MODE_2", 57},
    {"EXTERNAL_PORT_MODE_3", 58},
    {"STARTUP_CONFIGURATION", 60},
    {"SHUTDOWN", 63}
};


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
        ROS_INFO("\033[1;32mInitialization success\033[0m");
        return true;
    }
}

void dynamixelMotor::setControlTable()
{
    uint16_t *model_number = new uint16_t[1];
    uint8_t dxl_error;

    // Checking the DMXL model (adress 0 in all DMXLS)
    int dxl_comm_result = myPacketHandler->read2ByteTxRx(myPortHandler, this->ID, 0, model_number, &dxl_error);

    if(dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("DMXL %d: Failed to read the model number. Error code: %d",this->ID,dxl_error);
    } else {
        this->MODEL = static_cast<int>(*model_number); 

        switch(this->MODEL)
        {
            case 1180:
            case 1170:
            case 1280:
            case 1270:
            case 1011:
            case 1001:
            case 1010:
            case 1000:
            case 1050:
            case 1040:
            case 1030:
            case 1020:
            case 1160:
            case 1070:
            case 1080:
            case 1220:
            case 1210:
            case 1230:
            case 1240:
            case 1060:
            case 1190:
            case 1200:
            case 350:
                this->CONTROL_TABLE = ADDR_DMXL22;
            break;

            case 1111: 
            case 1101:
            case 1110:
            case 1100:
            case 1150:
            case 1140:
            case 1130:
            case 1120:
                this->CONTROL_TABLE = ADDR_DMXL25;
            break;

            // IN CASE OF UNRECOGNIZED MODEL NUMBER
            default:
                this->MODEL = 0;
            break;
        }

        ROS_INFO("\033[1;35mDMXL %d\033[0m: Control table set for: %s",this->ID, dynamixelMotor::DMXL_MODELS[this->MODEL].c_str());
    }

    delete[] model_number;
}

int dynamixelMotor::getID()
{
    return this->ID;
}

void dynamixelMotor::setID(int NEW_ID)
{
    // If ID is changed, please, pay attention to yours declarations. Now, you need to change the 'ID' parameter you use in dynamixelMotor constructor.
    int dxl_comm_result = 0;
    uint8_t dxl_error = 0;

    if(NEW_ID > 0 && NEW_ID < 254)
    {
        dxl_comm_result = myPacketHandler->write1ByteTxRx(myPortHandler,this->ID, this->CONTROL_TABLE["ID"], NEW_ID, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)    
        {
            ROS_ERROR("DMXL %d: Failed to change the ID. Error code: %d",this->ID, dxl_error);
        } else 
        {
            ROS_INFO("\033[1;35mDMXL %d\033[0m: Changed the ID. New ID: %d", this->ID, NEW_ID);
            this->ID = NEW_ID;
        }
    } else
    {
        ROS_ERROR("DMXL %d: Please, specify a valid DMXL ID(0 - 253)", this->ID);
    }

}

std::string dynamixelMotor::getModel()
{
    return dynamixelMotor::DMXL_MODELS[this->MODEL];
}

int dynamixelMotor::getBaudrate()
{
    uint8_t dxl_error = 0;
    uint8_t *data = new uint8_t[1];
    int baudrate;
    bool max_baudrate_45M = (this->MODEL == 1180 || this->MODEL == 1170 || this->MODEL == 1280 || this->MODEL == 1270 || 
                             this->MODEL == 1111 || this->MODEL == 1101 || this->MODEL == 1011 || this->MODEL == 1001 ||
                             this->MODEL == 1110 || this->MODEL == 1100 || this->MODEL == 1150 || this->MODEL == 1140 ||
                             this->MODEL == 1010 || this->MODEL == 1000 || this->MODEL == 1050 || this->MODEL == 1040 ||
                             this->MODEL == 1130 || this->MODEL == 1120 || this->MODEL == 1030 || this->MODEL == 1020 ||
                             this->MODEL == 1160 || this->MODEL == 1070 || this->MODEL == 1080 || this->MODEL == 1060);

    bool max_baudrate_4M = (this->MODEL == 1220 || this->MODEL == 1210 || this->MODEL == 1230 || this->MODEL == 1240 || this->MODEL == 1190 || this->MODEL == 1200);
    bool max_baudrate_1M = this->MODEL == 350;
    int dxl_comm_result = myPacketHandler->read1ByteTxRx(myPortHandler, this->ID, this->CONTROL_TABLE["BAUDRATE"], data, &dxl_error);
        
    if(dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("DMXL %d: Failed to read the baudrate. Error code: %d", this->ID, dxl_error);
        return -1;
    } else
    {
        if(max_baudrate_45M)
        {
            switch(static_cast<int>(*data))
            {
                case 0:
                    baudrate = 9600;
                break;

                case 1:
                    baudrate = 57600;
                break;

                case 2:
                    baudrate = 115200;
                break;

                case 3:
                    baudrate = 1e6;
                break;

                case 4:
                    baudrate = 2e6;
                break;

                case 5:
                    baudrate = 3e6;
                break;

                case 6:
                    baudrate = 4e6;
                break;

                case 7:
                    baudrate = 4.5e6;
                break;

                default:
                    baudrate = 0;
                break;
            }
        } else if(max_baudrate_4M)
        {
            switch (static_cast<int>(*data))
            {
                case 0:
                    baudrate = 9600;
                break;

                case 1:
                    baudrate = 57600;
                break;

                case 2:
                    baudrate = 115200;
                break;

                case 3:
                    baudrate = 1e6;
                break;

                case 4:
                    baudrate = 2e6;
                break;

                case 5:
                    baudrate = 3e6;
                break;

                case 6:
                    baudrate = 4e6;
                break;
            
                default:
                    baudrate = 0;
                break;
            }
        } else if(max_baudrate_1M)
        {
            switch (static_cast<int>(*data))
            {
                case 0:
                    baudrate = 9600;
                break;
            
                case 1:
                    baudrate = 57600;
                break;

                case 2:
                    baudrate = 115200;
                break;

                case 3:
                    baudrate = 1e6;
                break;

                default:
                    baudrate = 0;
                break;
            }
        }
        
        ROS_INFO("\033[1;35mDMXL %d\033[0m: Current baudrate is %d bps",this->ID,baudrate);
        delete[] data;
        return baudrate;
    }
}

int dynamixelMotor::getReturnDelayTime() //RETURNS MICRO SECONDS
{
    uint8_t dxl_error = 0;
    uint8_t *data = new uint8_t[1];
    int time;

    int dxl_comm_result = myPacketHandler->read1ByteTxRx(myPortHandler, this->ID, this->CONTROL_TABLE["RETURN_DELAY_TIME"], data, &dxl_error);
        
    if(dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("DMXL %d: Failed to read the return delay time. Error code: %d", this->ID,dxl_error);
        return -1;
    } else 
    {
        time = static_cast<int>(*data)*2;
        ROS_INFO("\033[1;35mDMXL %d\033[0m: Current return delay time is %d micro seconds",this->ID,time);
        return time;
    }

    delete[] data;
}

void dynamixelMotor::setReturnDelayTime(int RETURN_DELAY_TIME)
{
    int dxl_comm_result = 0;
    uint8_t dxl_error = 0;

    if(RETURN_DELAY_TIME > 0 && RETURN_DELAY_TIME < 508)
    {
        dxl_comm_result = myPacketHandler->write1ByteTxRx(myPortHandler,this->ID, this->CONTROL_TABLE["RETURN_DELAY_TIME"], RETURN_DELAY_TIME/2, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS)    
        {
            ROS_ERROR("DMXL %d: Failed to change the return delay time. Error code: %d", this->ID, dxl_error);
        } else 
        {
            ROS_INFO("\033[1;35mDMXL %d\033[0m: Return delay time changed to %d micro seconds.",this->ID,RETURN_DELAY_TIME);
        }
    } else
    {
        ROS_ERROR("DMXL %d: Please, specify a valid return delay time(0 - 508)",this->ID);
    }
}

void dynamixelMotor::showDriveModeConfig()
{
    uint8_t *config = new uint8_t[1];
    uint8_t dxl_error;

    int dxl_comm_result = myPacketHandler->read1ByteTxRx(myPortHandler, this->ID, this->CONTROL_TABLE["DRIVE_MODE"], config, &dxl_error);

    if(dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("DMXL %d: Failed to read the drive mode configuration. Error code: %d",this->ID,dxl_error);
    } else 
    {
        bool reverse_mode = (*config & 0b00000001) != 0;
        bool slave_mode = (*config & 0b00000010) != 0;
        bool time_based_profile = (*config & 0b00000100) != 0;
        bool auto_torque = (*config & 0b00001000) != 0;

        std::string s_reverse_mode, s_slave_mode, s_time_based_profile, s_torque_auto_on;
        s_reverse_mode = reverse_mode ? "ON" : "OFF";
        s_slave_mode = slave_mode ? "ON" : "OFF";
        s_time_based_profile = time_based_profile ? "ON" : "OFF";
        s_torque_auto_on = auto_torque ? "ON" : "OFF";    

        ROS_INFO("\n \033[0;35mCurrent config\033[0m \n REVERSE_MODE: %s \n SLAVE MODE: %s \n TIME BASED PROFILE: %s \n AUTO TORQUE: %s",
        s_reverse_mode.c_str(), s_slave_mode.c_str(), s_time_based_profile.c_str(), s_torque_auto_on.c_str());
    }

    delete[] config;
}

void dynamixelMotor::configDriveMode(bool REVERSE_MODE, bool SLAVE_MODE, bool TIME_BASED_PROFILE, bool TORQUE_AUTO_ON)
{
    // FOR MORE INFO ABOUT PARAMS, CHECK DYNAMIXEL SDK
    std::string s_reverse_mode, s_slave_mode, s_time_based_profile, s_torque_auto_on;
    s_reverse_mode = REVERSE_MODE ? "ON" : "OFF";
    s_slave_mode = SLAVE_MODE ? "ON" : "OFF";
    s_time_based_profile = TIME_BASED_PROFILE ? "ON" : "OFF";
    s_torque_auto_on = TORQUE_AUTO_ON ? "ON" : "OFF";
    uint8_t dxl_error = 0;
    uint8_t data = 0;

    if (REVERSE_MODE)
    {
        data |= (1 << 0);  // Set bit 0
    }

    if (SLAVE_MODE)
    {
        data |= (1 << 1);  // Set bit 1
    }

    if (TIME_BASED_PROFILE)
    {
        data |= (1 << 2);  // Set bit 2
    }

    if (TORQUE_AUTO_ON)
    {
        data |= (1 << 3);  // Set bit 3
    }

    int dxl_comm_result = myPacketHandler->write1ByteTxRx(myPortHandler,this->ID, this->CONTROL_TABLE["DRIVE_MODE"], data, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)    
    {
        ROS_ERROR("DMXL %d: Failed to change the drive mode configuration. Error code: %d", this->ID, dxl_error);
    } else 
    {
        ROS_INFO("\033[1;35mDMXL %d\033[0m: Drive mode configuration changed.",this->ID);      
        this->showDriveModeConfig();
    }

}

std::string dynamixelMotor::getOperatingMode()
{
    uint8_t *op_mode = new uint8_t[1];
    uint8_t dxl_error;

    int dxl_comm_result = myPacketHandler->read1ByteTxRx(myPortHandler, this->ID, this->CONTROL_TABLE["OPERATING_MODE"], op_mode, &dxl_error);

    if(dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("DMXL %d: Failed to read the operating mode. Error code: %d",this->ID,dxl_error);
        return nullptr;
    } else 
    {
        switch (static_cast<int>(*op_mode))
        {
            case dynamixelMotor::CURRENT_CONTROL_MODE:
                return "Current Control";
            break;

            case dynamixelMotor::VELOCITY_CONTROL_MODE:
                return "Velocity Control";
            break;

            case dynamixelMotor::POSITION_CONTROL_MODE:
                return "Position Control";
            break;

            case dynamixelMotor::EXTENDED_POSITION_CONTROL_MODE:
                return "Extenden Position Control";
            break;

            case dynamixelMotor::CURRENT_BASED_POSITION_CONTROL:
                return "Current Based Position Control";
            break;

            case dynamixelMotor::PWM_CONTROL_MODE:
                return "PWM Control";
            break;
        
            default:
                return "Unknown mode";
            break;
        }
    }

    delete[] op_mode;
}

void dynamixelMotor::setOperatingMode(int MODE)
{
    int dxl_comm_result = 0;
    uint8_t dxl_error = 0;
    bool modeExists = (MODE == dynamixelMotor::CURRENT_CONTROL_MODE || MODE == dynamixelMotor::VELOCITY_CONTROL_MODE ||
                       MODE == dynamixelMotor::POSITION_CONTROL_MODE || MODE == dynamixelMotor::EXTENDED_POSITION_CONTROL_MODE ||
                       MODE == dynamixelMotor::EXTENDED_POSITION_CONTROL_MODE || MODE == dynamixelMotor::PWM_CONTROL_MODE);

    if(modeExists)
    {
        dxl_comm_result = myPacketHandler->write1ByteTxRx(myPortHandler,this->ID, this->CONTROL_TABLE["OPERATING_MODE"], MODE, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS)    
        {
            ROS_ERROR("DMXL %d: Failed to change the operatin mode. Error code: %d",this->ID, dxl_error);
        } else 
        {
            ROS_INFO("\033[1;35mDMXL %d\033[0m: Changed the operating mode.", this->ID);
        }

    } else
    {
        ROS_ERROR("DMXL %d: Please, specify a valid DMXL operating mode.", this->ID);
    }
}

int dynamixelMotor::getShadowID()
{
    uint8_t dxl_error = 0;
    uint8_t *data = new uint8_t[1];

    int dxl_comm_result = myPacketHandler->read1ByteTxRx(myPortHandler, this->ID, this->CONTROL_TABLE["SECONDARY_ID"], data, &dxl_error);
        
    if(dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("DMXL %d: Failed to read the secondary ID. Error code: %d", this->ID, dxl_error);
        return -1;
    } else 
    {
        int shadow_id = static_cast<int>(*data);

        if(shadow_id > 252)
        {
            ROS_INFO("\033[1;35mDMXL %d\033[0m: Current shadow id: %d (disabled) ",this->ID,shadow_id);
        } else 
        {
            ROS_INFO("\033[1;35mDMXL %d\033[0m: Current shadow id: %d",this->ID,shadow_id);
        }
        return shadow_id;
    }

    delete[] data;
}

void dynamixelMotor::setShadowID(int NEW_SH_ID)
{
    // Before using, please learn about shadow (secondary) ID and its usage.
    uint8_t dxl_error = 0;

    if(NEW_SH_ID >= 0 && NEW_SH_ID < 256)
    {
        int dxl_comm_result = myPacketHandler->write1ByteTxRx(myPortHandler,this->ID, this->CONTROL_TABLE["SECONDARY_ID"], NEW_SH_ID, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS)    
        {
            ROS_ERROR("DMXL %d: Failed to change the secondary ID. Error code: %d",this->ID, dxl_error);
        } else 
        {
            if(NEW_SH_ID == 253 || NEW_SH_ID == 254 || NEW_SH_ID == 255)
            {
                ROS_INFO("\033[1;35mDMXL %d\033[0m: Secondary ID was set to %d (disabled).", this->ID, NEW_SH_ID);

            } else {
                ROS_INFO("\033[1;35mDMXL %d\033[0m: Changed the secondary ID. New secondary ID: %d", this->ID, NEW_SH_ID);
            }
        }

    } else
    {
        ROS_ERROR("DMXL %d: Please, specify a valid DMXL SHADOW ID(0 - 255)", this->ID);
    }
}

int dynamixelMotor::getProcotolType()
{
    uint8_t dxl_error = 0;
    uint8_t *data = new uint8_t[1];

    int dxl_comm_result = myPacketHandler->read1ByteTxRx(myPortHandler, this->ID, this->CONTROL_TABLE["PROTOCOL_TYPE"], data, &dxl_error);
        
    if(dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("DMXL %d: Failed to read the protocol type. Error code: %d", this->ID, dxl_error);
        return -1;
    } else 
    {
        ROS_INFO("\033[1;35mDMXL %d\033[0m: Current protocol type: %d.0",this->ID,static_cast<int>(*data));
        return static_cast<int>(*data);
    }

    delete[] data;
}

int dynamixelMotor::getHomingOffset()
{
    uint8_t dxl_error = 0;
    uint32_t *data = new uint32_t[1];

    int dxl_comm_result = myPacketHandler->read4ByteTxRx(myPortHandler, this->ID, this->CONTROL_TABLE["HOMING_OFFSET"], data, &dxl_error);
        
    if(dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("DMXL %d: Failed to read the homing offset. Error code: %d", this->ID, dxl_error);
        return -1;
    } else 
    {
        double degrees = static_cast<double>(*data)* 0.088;
        ROS_INFO("\033[1;35mDMXL %d\033[0m: Current homing offset is: %.1f degrees",this->ID,degrees);
        return static_cast<int>(*data);
    }

    delete[] data;
}

void dynamixelMotor::setHomingOffset(int DEGREES)
{
    uint8_t dxl_error = 0;

    if((DEGREES/360) >= -255 && (DEGREES/360) <= 255)
    {
        int dxl_comm_result = myPacketHandler->write4ByteTxRx(myPortHandler,this->ID, this->CONTROL_TABLE["HOMING_OFFSET"], (DEGREES/0.088), &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS)    
        {
            ROS_ERROR("DMXL %d: Failed to change the homing offset. Error code: %d",this->ID, dxl_error);
        } else 
        {
            ROS_INFO("\033[1;35mDMXL %d\033[0m: Changed the homing offset. New homing offset: %d degrees", this->ID, DEGREES);
        }

    } else
    {
        ROS_ERROR("DMXL %d: Please, specify a valid HOMING OFFSET(between 255rev and -255rev)", this->ID);
    }
}

double dynamixelMotor::getMovingThreshold()
{
    uint8_t dxl_error = 0;
    uint32_t *data = new uint32_t[1];

    int dxl_comm_result = myPacketHandler->read4ByteTxRx(myPortHandler, this->ID, this->CONTROL_TABLE["MOVING_THRESHOLD"], data, &dxl_error);
        
    if(dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("DMXL %d: Failed to read the moving threshold. Error code: %d", this->ID, dxl_error);
        return -1;
    } else 
    {
        double RPM = static_cast<double>(*data) * 0.229;
        ROS_INFO("\033[1;35mDMXL %d\033[0m: Current moving threshold is: %.1f rpm",this->ID,RPM);
        return RPM;
    }

    delete[] data;
}

void dynamixelMotor::setMovingThreshold(double RPM)
{
    uint8_t dxl_error = 0;
    float conversion = 0.229;

    if(RPM >= 0 && RPM <= 235)
    {
        int dxl_comm_result = myPacketHandler->write4ByteTxRx(myPortHandler,this->ID, this->CONTROL_TABLE["MOVING_THRESHOLD"], (RPM/conversion), &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS)    
        {
            ROS_ERROR("DMXL %d: Failed to change the moving threshold. Error code: %d",this->ID, dxl_error);
        } else 
        {
            ROS_INFO("\033[1;35mDMXL %d\033[0m: Changed the moving threshold. New moving threshold: %.1f rpm", this->ID, RPM);
        }

    } else
    {
        ROS_ERROR("DMXL %d: Please, specify a valid moving threshold(between 0rpm and 235rpm)", this->ID);
    }
}

