#ifndef DYNAMIXEL_ROS_LIBRARY_H
#define DYNAMIXEL_ROS_LIBRARY_H

// DEPENDENCIES
#include <dynamixel_sdk.h>

// OPERATING MODES (idk if necessary)
/*enum class operatingMode{
    CURRENT_CONTROL = 0,
    VELOCITY_CONTROL = 1,
    POSITION_CONTROL = 3,
    EXTENDED_POSITION_CONTROL = 4,
    CURRENT_BASED_POSITION_CONTROL = 5,
    PWM_CONTROL = 16
};
*/

// CLASS DEFINITION
class dynamixelMotor
{
    public:
        // CONSTS
        static const int CURRENT_CONTROL_MODE = 0, 
                         VELOCITY_CONTROL_MODE = 1, 
                         POSITION_CONTROL_MODE = 3, 
                         EXTENDED_POSITION_CONTROL_MODE = 4, 
                         CURRENT_BASED_POSITION_CONTROL = 5, 
                         PWM_CONTROL_MODE = 16;

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
        void setBaudrate();

        int getReturnDelayTime();
        void setReturnDelayTime(int RETURN_DELAY_TIME);

        std::string getOperatingMode();
        void setOperatingMode(int MODE);

        int getShadowID();
        void setShadowID(int NEW_SH_ID);

        int getProcotolType();
        // void setProtocolType(int PROTOCOL_TYPE);  not implemented at the moment

        int getHomingOffset(); // returns degrees
        void setHomingOffset(int DEGREES);

        double getMovingThreshold();
        void setMovingThreshold(double RPM);

        int getTempLimit();    // REGISTERS 70 and 63 need to be modified to shutdown when T is too much
        void setTempLimit(int TEMPERATURE);

        float getMaxVoltageLimit();   //SAME
        void setMaxVoltageLimit(float MAX_VOLTAGE);

        float getMinVoltageLimit();
        void setMinVoltageLimit(float MIN_VOLTAGE);

        int getPWMLimit();
        void setPWMLimit(int PWM);

        float getCurrentLimit();
        void setCurrentLimit(float CURRENT_mA);
        //
        float getVelLimit();
        void setVelLimit(float VEL_LIMIT_RPM);

        float getMaxPosLimit();
        void setMaxPosLimit(float MAX_POS_LIMIT_DEGREES);
        
        float getMinPosLimit();
        void setMinPosLimit(float MIN_POS_LIMIT_DEGREES);

        bool getTorqueState();
        void setTorqueState(bool TORQUE_ENABLE);

        bool getLedState();
        void setLedState(bool LED_STATE);

        int getStatusReturnLevel();
        void setStatusReturnLevel(int STATUS_RETURN_LEVEL);

        std::vector<bool> getHardwareErrorStatus();

        void getVelocityPIValues(int &P, int &I);
        void setVelocityPIValues(int P, int I);

        void getPositionPIDValues(int &P, int &I, int &D);
        void setPositionPIDValues(int P, int I, int D);
        
        // Config methods
        void configDriveMode(bool REVERSE_MODE, bool SLAVE_MODE, bool TIME_BASED_PROFILE, bool TORQUE_AUTO_ON);
        void showDriveModeConfig();

        void configStartup(bool TORQUE_ON, bool RAM_RESTORE); //try with other motor
        void showStartupConfig();

        void configShutdown(bool INPUT_VOLTAGE_ERROR, bool OVERHEATING_ERROR, bool ENCODER_ERROR, bool ELECTRICAL_SHOCK_ERROR, bool OVERLOAD_ERROR);
        void showShutdownConfig();
        
    private:
        // MAPS USED TO CHANGE BETWEEN DIFF EEPROM CONTROL TABLES
        static std::map<std::string, int> ADDR_DMXL22, ADDR_DMXL25;

        // MAPS THAT CONECTS A MODELS WITH ITS MODEL NUMBERS
        static std::map<int, std::string> DMXL_MODELS;

        // Dynamixels parameters
        int ID;
        int MODEL;
        std::string IDENTIFICATOR;
        std::map<std::string, int> CONTROL_TABLE;
};

#endif