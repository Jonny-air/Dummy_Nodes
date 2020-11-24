/**
 * @file dummy_offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "dummy_offb_node");
    ros::NodeHandle nh;
    bool use_local = nh.param<bool>("dummy_offb_node/use_local", use_local, "false");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
          ("mavros/setpoint_position/local", 10);

    ros::Publisher global_pos_pub = nh.advertise<geographic_msgs::GeoPoseStamped>
          ("mavros/setpoint_position/global_to_local", 10);
    //
    // ros::Publisher set_gp_origin_pub = nh.advertise<geographic_msgs::GeoPointStamped>
    //       ("mavros/global_position/set_gp_origin", 10);

    // ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    //         ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
          ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //local origin
    // geographic_msgs::GeoPointStamped geo_origin;
    // geo_origin.position.latitude = 47.3666999;
    // geo_origin.position.longitude = 8.5501323;
    // geo_origin.position.altitude = 500;


    geometry_msgs::PoseStamped pose; //gets transformed to localPosition frame x--y flipped and z = -z
    pose.pose.position.x = 723029; //(y)
    pose.pose.position.y = 5282566; //(x)
    pose.pose.position.z = -500.2; //(-z)
    // pose.pose.position.x = 0;
    // pose.pose.position.y = 0;
    // pose.pose.position.z = -1;


    geographic_msgs::GeoPoseStamped geopose;
    geopose.pose.position.latitude = 47.3666999;
    geopose.pose.position.longitude = 8.5501323;
    geopose.pose.position.altitude = 501;



    //set desired origin for local frame
    // for(int i = 1; ros::ok() && i > 0; --i){
    //     set_gp_origin_pub.publish(geo_origin);
    //
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    //send a few setpoints before starting
    for(int i = 10; ros::ok() && i > 0; --i){
        if (use_local) {
          local_pos_pub.publish(pose);
        } else {
          global_pos_pub.publish(geopose);
        }

        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    //
    // mavros_msgs::CommandBool arm_cmd;
    // arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        // else {
        //     if( !current_state.armed &&
        //         (ros::Time::now() - last_request > ros::Duration(5.0))){
        //         if( arming_client.call(arm_cmd) &&
        //             arm_cmd.response.success){
        //             ROS_INFO("Vehicle armed");
        //         }
        //         last_request = ros::Time::now();
        //     }
        // }
        if (use_local) {
          local_pos_pub.publish(pose);
        } else {
          global_pos_pub.publish(geopose);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
