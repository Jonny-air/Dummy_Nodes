/**
 * @file dummy_offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <stdlib.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "dummy_offb_node_attitude");
    ros::NodeHandle nh;

    //Waypoints
    ros::NodeHandle p;
    ros::NodeHandle n;

    //


    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Publisher attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>
          ("mavros/setpoint_raw/attitude", 10);

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
          ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //Attitude Message
    mavros_msgs::AttitudeTarget att_msg;

    // msg.header.stamp = ros::Time::now();
    // msg.header.frame_id = "map";
    att_msg.body_rate.x = 0;
    att_msg.body_rate.y = 0;
    att_msg.body_rate.z = 0.6;
    att_msg.type_mask = 3;  // Ignore roll and pitch rate
    att_msg.orientation.w = 0.0;
    att_msg.orientation.x = 0.0;
    att_msg.orientation.y = 0.0;
    att_msg.orientation.z =  -1.0;
    att_msg.thrust = 69;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    //send a few setpoints before starting
    for(int i = 10; ros::ok() && i > 0; --i){
        attitude_pub.publish(att_msg);
        ros::spinOnce();
        rate.sleep();
    }


    ros::Time last_request = ros::Time::now();

    if(current_state.mode != "OFFBOARD"){
        if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
    }

    while(ros::ok()){
        if(current_state.mode != "OFFBOARD" && ros::Time::now() - last_request > ros::Duration(5.0))
        {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }

        attitude_pub.publish(att_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
