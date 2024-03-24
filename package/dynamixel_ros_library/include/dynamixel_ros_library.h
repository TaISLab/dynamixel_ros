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
        void setID(int NEW_ID);

        std::string getModel();

        int getBaudrate();
        
    private:
        static std::map<std::string, int> ADDR_DMXL22, ADDR_DMXL25;
        static std::map<int, std::string> DMXL_MODELS;
        std::map<std::string, int> CONTROL_TABLE;

        // Dynamixels parameters
        int ID;
        int MODEL;
        std::string IDENTIFICATOR;
};

#endif