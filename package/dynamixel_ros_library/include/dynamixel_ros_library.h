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

        // Tables initialization and setting
        void setControlTable();

        // Getters and setters
        int getID();
        void setID();

        std::string getModel();

        int getBaudrate();

    private:
        static std::map<std::string, int> ADDR_XW_SERIES, ADDR_XD540_SERIES, ADDR_XD430_SERIES;
        std::map<std::string, int> CURRENT_TABLE;

        // Dynamixels parameters
        int ID;
        std::string MODEL;
        std::string IDENTIFICATOR;
};

#endif