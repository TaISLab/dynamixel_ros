#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <dynamixel_ros_library.h>
#include <iostream>

dynamixelMotor motorJ0, motorJ1, motorJ2, motorJ10, motorJ11, motorJ12;

int main(int argc, char *argv[])
{
    // Define port, rate and protocol
    // Default values
    char *port_name = "/dev/ttyUSB0";
    int baud_rate = 4500000;
    float protocol_version = 2.0;
    // char* port_name;
    // int baud_rate;
    // float protocol_version;

    // if (argc != 4)
    // {
    //     printf("Please set '-port_name', '-protocol_version' '-baud_rate' arguments for connected Dynamixels\n");
    //     return 0;
    // } else
    // {
    //     port_name = argv[1];
    //     protocol_version = atoi(argv[2]);
    //     baud_rate = atoi(argv[3]);
    // }

    // Init communication
    dynamixelMotor::iniComm(port_name, protocol_version, baud_rate);
    motorJ0 = dynamixelMotor("J0", 0);
    motorJ10 = dynamixelMotor("J10", 10);
    motorJ1 = dynamixelMotor("J1", 1);
    motorJ11 = dynamixelMotor("J11", 11);
    motorJ2 = dynamixelMotor("J2", 2);
    motorJ12 = dynamixelMotor("J12", 12);

    // Set control table
    motorJ0.setControlTable();
    motorJ10.setControlTable();
    motorJ1.setControlTable();
    motorJ11.setControlTable();
    motorJ2.setControlTable();
    motorJ12.setControlTable();

    // Define the control mode for each motor
    motorJ0.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
    motorJ1.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
    motorJ2.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
    motorJ10.setOperatingMode(dynamixelMotor::VELOCITY_CONTROL_MODE);
    motorJ11.setOperatingMode(dynamixelMotor::VELOCITY_CONTROL_MODE);
    motorJ12.setOperatingMode(dynamixelMotor::VELOCITY_CONTROL_MODE);

    // Set joint velocity limit
    float MAX_VELOCITY = 45.0;
    motorJ10.setVelLimit(MAX_VELOCITY);
    motorJ11.setVelLimit(MAX_VELOCITY);
    motorJ12.setVelLimit(MAX_VELOCITY);

    // Set joint position limits (change the numbers for your gripper)
    motorJ0.setMaxPosLimit(242.0);
    motorJ1.setMaxPosLimit(348.0);
    motorJ2.setMinPosLimit(342.0);
    motorJ0.setMinPosLimit(93.96);
    motorJ1.setMinPosLimit(218.23);
    motorJ2.setMinPosLimit(137.0);

    // Enable Torque
    motorJ0.setTorqueState(true);
    motorJ1.setTorqueState(true);
    motorJ2.setTorqueState(true);
    motorJ10.setTorqueState(true);
    motorJ11.setTorqueState(true);
    motorJ12.setTorqueState(true);

    // Open and closed joint values
    float motor0_open = 242.0;
    float motor1_open = 218.23;
    float motor2_open = 342.0;
    float motor0_closed = 93.96;
    float motor1_closed = 348.0;
    float motor2_closed = 137.0;

    // Velocity values
    float slow_velocity = 20;
    float normal_velocity = 30;
    float fast_velocity = 40;

    // State 0: gripper open, not rotating
    motorJ0.setGoalPosition(motor0_open);
    motorJ1.setGoalPosition(motor1_open);
    motorJ2.setGoalPosition(motor2_open);
    motorJ10.setGoalVelocity(0);
    motorJ11.setGoalVelocity(0);
    motorJ12.setGoalVelocity(0);

    // ROS node init
    ros::init(argc, argv, "Ball_rolling");
    ros::NodeHandle nh;

    // ROS freq = 100 Hz
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        int state;
        // Wait for user to hit a key
        printf("Select the next state: %d\n", state);
        printf("0: Gripper open, not rotating\n");
        printf("1: Gripper closed, not rotating\n");
        printf("2: Gripper closed, rotating\n");
        std::cin >> state;
        if (state != 0 || state != 1 || state != 2)
        {
            printf("Wrong state selection\n");
        }
        else
        {
            printf("Going to state: %c\n", state);
        }
        switch (state)
        {
        case 0:
            // State 0: gripper open, not rotating
            motorJ0.setGoalPosition(motor0_open);
            motorJ1.setGoalPosition(motor1_open);
            motorJ2.setGoalPosition(motor2_open);
            motorJ10.setGoalVelocity(0);
            motorJ11.setGoalVelocity(0);
            motorJ12.setGoalVelocity(0);
            break;
        case 1:
            // State 1: gripper closed, notrotating
            motorJ0.setGoalPosition(motor0_closed);
            motorJ1.setGoalPosition(motor1_closed);
            motorJ2.setGoalPosition(motor2_closed);
            motorJ10.setGoalVelocity(0);
            motorJ11.setGoalVelocity(0);
            motorJ12.setGoalVelocity(0);
            break;
        case 2:
            // State 2: gripper closed, rotating
            // State 1: gripper closed, notrotating
            motorJ0.setGoalPosition(motor0_closed);
            motorJ1.setGoalPosition(motor1_closed);
            motorJ2.setGoalPosition(motor2_closed);
            ros::Duration(0.5).sleep(); // sleep for half a second (wait for closing)
            // Start rotation
            ros::Time rotation_begin = ros::Time::now();
            ros::Duration rotation_rate_half(5.0);
            ros::Duration rotation_rate_full(10.0);

            do
            {
                ros::Time rotation_time = ros::Time::now();
                if (rotation_time - rotation_begin < rotation_rate_half)
                {
                    motorJ10.setGoalVelocity(normal_velocity);
                    motorJ11.setGoalVelocity(normal_velocity);
                    motorJ12.setGoalVelocity(normal_velocity);
                }
                else if (rotation_time - rotation_begin > rotation_rate_half && rotation_time - rotation_begin < rotation_rate_full)
                {
                    motorJ10.setGoalVelocity(-normal_velocity);
                    motorJ11.setGoalVelocity(-normal_velocity);
                    motorJ12.setGoalVelocity(-normal_velocity);
                }
                else
                {
                    ros::Time rotation_begin = ros::Time::now();
                }
            } while (std::cout << "press a key to exit the rotation loop");
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
