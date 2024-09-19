#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <dynamixel_ros_library.h>
#include <iostream>

void publishFSMState(int &state, ros::Publisher &fsm_pub)
{
    // Creating MSG objects
    std_msgs::Int16 fsm_state;

    // data assignation
    fsm_state.data = state;

    // Publishing
    fsm_pub.publish(fsm_state);
}

int main(int argc, char *argv[])
{
    int state;
    int prev_state = 0;

    // ROS node init
    ros::init(argc, argv, "fsm_3_fingers_node");
    ros::NodeHandle nh;

    // Publishers and subscribers creation
    ros::Publisher fsm_state_publisher = nh.advertise<std_msgs::Int16>("fsm_state_3fingers", 10);

    // ROS freq = 100 Hz
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        // Wait for user to hit a key
        printf("Select the next state:\n");
        printf("0: Gripper open, no torque\n");
        printf("1: Gripper open, torque enabled\n");
        printf("2: Gripper closed, not rotating\n");
        printf("3: Gripper closed, rotating\n");
        std::cin >> state;
        if (state != 0 && state != 1 && state != 2 && state != 3)
        {
            printf("Wrong state selection\n");
        }
        else
        {
            printf("Going to state: %d\n", state);
            if (state != prev_state)
            {
                publishFSMState(state, fsm_state_publisher);
            }

            prev_state = state;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
