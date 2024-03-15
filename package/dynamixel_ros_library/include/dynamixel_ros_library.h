#ifndef DYNAMIXEL_ROS_LIBRARY_H
#define DYNAMIXEL_ROS_LIBRARY_H

// DEPENDENCIES
#include <dynamixel_sdk.h>

// CLASS DEFINITION
class dynamixelMotor
{
    public:
        // CONSTRUCTOR AND DESTRUCTOR
        dynamixelMotor(std::string IDENTIFICATOR, int ID);
        ~dynamixelMotor();

        // Communication start
        static bool iniComm(char* PORT_NAME, float PROTOCOL_VERSION, int BAUDRATE);

        // Setting the control table
        void setControlTable();

        // Getters and setters
        int getID();
        void setID();

        std::string getModel();

        int getBaudrate();

    private:
        // enum ADDR_X_SERIES
        // {
        //     // EEPROM
        //     MODEL_NUMBER = 0,
        //     MODEL_INFORMATION = 2,
        //     FIRMWARE_VERSION = 6,
        //     ID = 7,
        //     BAUDRATE = 8,
        //     RETURN_DELAY_TIME = 9,
        //     DRIVE_MODE = 10,
        //     OPERATING_MODE = 11,
        //     SECONDARY_ID = 12,
        //     PROTOCOL_TYPE = 13,
        //     HOMING_OFFSET = 20,
        //     MOVING_THRESHOLD = 24,
        //     TEMPERATURE_LIMIT = 31,
        //     MAX_VOLTAGE_LIMIT = 32,
        //     MIN_VOLTAGE_LIMIT = 34,
        //     PWM_LIMIT = 36,
        //     CURRENT_LIMIT = 38,
        //     VELOCITY_LIMIT = 44,
        //     MAX_POSITION_LIMIT = 48,
        //     MIN_POSITION_LIMIT = 52,
        //     STARTUP_CONFIGURATION = 60,
        //     SHUTDOWN = 63

        //     // RAM
        // };

        // Dynamixels parameters
        int ID;
        std::string MODEL;
        std::string IDENTIFICATOR;
};

#endif