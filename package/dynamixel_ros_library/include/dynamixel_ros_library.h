#ifndef DYNAMIXEL_ROS_LIBRARY_H
#define DYNAMIXEL_ROS_LIBRARY_H

// DEPENDENCIES
#include <dynamixel_sdk.h>

// CLASS DEFINITION
class dynamixelMotor
{
    public:
        // CONSTRUCTOR AND DESTRUCTOR
        dynamixelMotor(std::string IDENTIFICATOR,std::string MODEL, int ID);
        ~dynamixelMotor();

        // Communication start
        static bool iniComm(char* PORT_NAME, float PROTOCOL_VERSION, int BAUDRATE);

        // Getters and setters
        int getMotorID();
        void setMotorID();

    private:
        int ID;
        std::string MODEL;
        std::string IDENTIFICATOR;

};

#endif