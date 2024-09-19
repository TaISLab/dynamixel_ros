#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <dynamixel_ros_library.h>
#include <iostream>

dynamixelMotor motorJ0, motorJ1, motorJ2, motorJ10, motorJ11, motorJ12;
int fsm_state = 0;

void publishMotorStatus(dynamixelMotor &motor, ros::Publisher &pos_pub, ros::Publisher &vel_pub, ros::Publisher &curr_pub)
{
    // Creating MSG objects
    std_msgs::Float32 pos_msg;
    std_msgs::Float32 vel_msg;
    std_msgs::Float32 curr_msg;

    // Getting params from dmxl
    float position = (float)(motor.getPresentPosition());
    float velocity = (float)(motor.getPresentVelocity());
    float current = (float)(motor.getPresentCurrent());

    // data assignation
    pos_msg.data = position;
    vel_msg.data = velocity;
    curr_msg.data = current;

    // Publishing
    pos_pub.publish(pos_msg);
    vel_pub.publish(vel_msg);
    curr_pub.publish(curr_msg);
}

// Callback when some data was published in 'pos_user_input'
void fsmStateCallBack(const std_msgs::Int16::ConstPtr &msg)
{
    fsm_state = msg->data;
}

void torqueEnabled()
{
    motorJ0.setTorqueState(true);
    motorJ1.setTorqueState(true);
    motorJ2.setTorqueState(true);
    motorJ10.setTorqueState(true);
    motorJ11.setTorqueState(true);
    motorJ12.setTorqueState(true);
}

void torqueDisabled()
{
    motorJ0.setTorqueState(false);
    motorJ1.setTorqueState(false);
    motorJ2.setTorqueState(false);
    motorJ10.setTorqueState(false);
    motorJ11.setTorqueState(false);
    motorJ12.setTorqueState(false);
}

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
    motorJ1 = dynamixelMotor("J1", 1);
    motorJ2 = dynamixelMotor("J2", 2);
    motorJ10 = dynamixelMotor("J10", 10);
    motorJ11 = dynamixelMotor("J11", 11);
    motorJ12 = dynamixelMotor("J12", 12);

    // Set control table
    motorJ0.setControlTable();
    motorJ1.setControlTable();
    motorJ2.setControlTable();
    motorJ10.setControlTable();
    motorJ11.setControlTable();
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
    motorJ1.setMaxPosLimit(221.0);
    motorJ2.setMaxPosLimit(342.0);
    motorJ0.setMinPosLimit(2);
    motorJ1.setMinPosLimit(1);
    motorJ2.setMinPosLimit(105.0);

    // Enable Torque
    torqueEnabled();

    // Open and closed joint values
    float motor0_open = 242.0;
    float motor1_open = 1;
    float motor2_open = 342;
    float motor0_closed = 2;
    float motor1_closed = 221;
    float motor2_closed = 105;

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

    // Publishers and subscribers creation
    ros::Publisher J0_pos_publisher = nh.advertise<std_msgs::Float32>("J0_position", 10);
    ros::Publisher J0_vel_publisher = nh.advertise<std_msgs::Float32>("J0_velocity", 10);
    ros::Publisher J0_curr_publisher = nh.advertise<std_msgs::Float32>("J0_current", 10);
    ros::Publisher J1_pos_publisher = nh.advertise<std_msgs::Float32>("J1_position", 10);
    ros::Publisher J1_vel_publisher = nh.advertise<std_msgs::Float32>("J1_velocity", 10);
    ros::Publisher J1_curr_publisher = nh.advertise<std_msgs::Float32>("J1_current", 10);
    ros::Publisher J2_pos_publisher = nh.advertise<std_msgs::Float32>("J2_position", 10);
    ros::Publisher J2_vel_publisher = nh.advertise<std_msgs::Float32>("J2_velocity", 10);
    ros::Publisher J2_curr_publisher = nh.advertise<std_msgs::Float32>("J2_current", 10);
    ros::Publisher J10_pos_publisher = nh.advertise<std_msgs::Float32>("J10_position", 10);
    ros::Publisher J10_vel_publisher = nh.advertise<std_msgs::Float32>("J10_velocity", 10);
    ros::Publisher J10_curr_publisher = nh.advertise<std_msgs::Float32>("J10_current", 10);
    ros::Publisher J11_pos_publisher = nh.advertise<std_msgs::Float32>("J11_position", 10);
    ros::Publisher J11_vel_publisher = nh.advertise<std_msgs::Float32>("J11_velocity", 10);
    ros::Publisher J11_curr_publisher = nh.advertise<std_msgs::Float32>("J11_current", 10);
    ros::Publisher J12_pos_publisher = nh.advertise<std_msgs::Float32>("J12_position", 10);
    ros::Publisher J12_vel_publisher = nh.advertise<std_msgs::Float32>("J12_velocity", 10);
    ros::Publisher J12_curr_publisher = nh.advertise<std_msgs::Float32>("J12_current", 10);

    ros::Subscriber fsm_state_subscriber = nh.subscribe("fsm_state_3fingers", 10, fsmStateCallBack);

    // ROS freq = 100 Hz
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        // // Wait for user to hit a key
        // printf("Select the next state: %d\n", state);
        // printf("0: Gripper open, not rotating\n");
        // printf("1: Gripper closed, not rotating\n");
        // printf("2: Gripper closed, rotating\n");
        // std::cin >> state;
        // if (state != 0 || state != 1 || state != 2)
        // {
        //     printf("Wrong state selection\n");
        // }
        // else
        // {
        //     printf("Going to state: %c\n", state);
        // }

        switch (fsm_state)
        {
        case 0:
                // State 0: gripper open, not rotating
                motorJ0.setGoalPosition(motor0_open);
                motorJ1.setGoalPosition(motor1_open);
                motorJ2.setGoalPosition(motor2_open);
                motorJ10.setGoalVelocity(0);
                motorJ11.setGoalVelocity(0);
                motorJ12.setGoalVelocity(0);
                ros::Duration(0.8).sleep(); // sleep for half a second (wait for opening)
                torqueDisabled();
            break;
        case 1:
                torqueEnabled();
                // State 0: gripper open, not rotating
                motorJ0.setGoalPosition(motor0_open);
                motorJ1.setGoalPosition(motor1_open);
                motorJ2.setGoalPosition(motor2_open);
                motorJ10.setGoalVelocity(0);
                motorJ11.setGoalVelocity(0);
                motorJ12.setGoalVelocity(0);
            break;
        case 2:
                torqueEnabled();
                // State 1: gripper closed, notrotating
                motorJ0.setGoalPosition(motor0_closed);
                motorJ1.setGoalPosition(motor1_closed);
                motorJ2.setGoalPosition(motor2_closed);
                motorJ10.setGoalVelocity(0);
                motorJ11.setGoalVelocity(0);
                motorJ12.setGoalVelocity(0);
            break;
        case 3:
            torqueEnabled();
            // State 2: gripper closed, rotating
            // State 1: gripper closed, notrotating
            motorJ0.setGoalPosition(motor0_closed);
            motorJ1.setGoalPosition(motor1_closed);
            motorJ2.setGoalPosition(motor2_closed);
            ros::Duration(0.8).sleep(); // sleep for half a second (wait for closing)
            // Start rotation
            ros::Time rotation_begin = ros::Time::now();
            ros::Duration rotation_rate_half(0.5);
            ros::Duration rotation_rate_full(1.0);

            ros::Time rotation_time = ros::Time::now();
            std::cout << rotation_time << std::endl;
            if (rotation_time - rotation_begin < rotation_rate_half)
            {
                std::cout << "one direction" << std::endl;
                motorJ10.setGoalVelocity(-slow_velocity);
                motorJ11.setGoalVelocity(-slow_velocity);
                motorJ12.setGoalVelocity(-slow_velocity);
            }
            else if (rotation_time - rotation_begin > rotation_rate_half && rotation_time - rotation_begin < rotation_rate_full)
            {
                std::cout << "the other direction" << std::endl;
                motorJ10.setGoalVelocity(slow_velocity);
                motorJ11.setGoalVelocity(slow_velocity);
                motorJ12.setGoalVelocity(slow_velocity);
            }
            else
            {
                ros::Time rotation_begin = ros::Time::now();
            }

            break;
        }

        publishMotorStatus(motorJ0, J0_pos_publisher, J0_vel_publisher, J0_curr_publisher);
        publishMotorStatus(motorJ1, J1_pos_publisher, J1_vel_publisher, J1_curr_publisher);
        publishMotorStatus(motorJ2, J2_pos_publisher, J2_vel_publisher, J2_curr_publisher);
        publishMotorStatus(motorJ10, J10_pos_publisher, J10_vel_publisher, J10_curr_publisher);
        publishMotorStatus(motorJ11, J11_pos_publisher, J11_vel_publisher, J11_curr_publisher);
        publishMotorStatus(motorJ12, J12_pos_publisher, J12_vel_publisher, J12_curr_publisher);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
