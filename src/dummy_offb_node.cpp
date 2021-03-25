/**
 * @file dummy_offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <stdlib.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

//Waypoints
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <std_msgs/String.h>
#include <cstdlib>
#include <mavros_msgs/Waypoint.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pos;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  current_pos = *msg;
}

float _last_gnd_D = 0.0f;
bool _is_climbout = false;
bool _waypoint_received = false;
bool _was_below = false;
bool _climbout_complete = true;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "dummy_offb_node");
    ros::NodeHandle nh;

    //Waypoints
    ros::NodeHandle p;
    ros::NodeHandle n;

    ros::ServiceClient wp_clear_client = p.serviceClient<mavros_msgs::WaypointClear>("/mavros/mission/clear");
    ros::ServiceClient wp_srv_client = n.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");

    mavros_msgs::WaypointPush wp_push_srv;
    mavros_msgs::WaypointClear wp_clear_srv;

    //

    bool add_noise = false;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pos_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
          ("mavros/setpoint_position/local", 10);

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
          ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //Waypoint
    mavros_msgs::Waypoint wp_msg;

    wp_msg.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp_msg.command = 17;
    wp_msg.is_current = true;
    wp_msg.autocontinue = true;
    wp_msg.param1 = 0;
    wp_msg.param2 = 0;
    wp_msg.param3 = 0;
    wp_msg.param4 = 0;
    wp_msg.x_lat = 0;
    wp_msg.y_long = -0.00329 ;
    wp_msg.z_alt = 40.0;

    geometry_msgs::PoseStamped target_pose; //gets transformed to localPosition frame x--y flipped and z = -z
    geometry_msgs::PoseStamped true_pose;

    target_pose.pose.position.x = 1000; //(y)
    target_pose.pose.position.y = 2000; //(x)
    target_pose.pose.position.z = 502; //(-z) for state level

    true_pose = target_pose;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::SetMode auto_set_mode;
    auto_set_mode.request.custom_mode = "AUTO.MISSION";

    srand48(time(NULL));

    //send a few setpoints before starting
    for(int i = 10; ros::ok() && i > 0; --i){
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }


    ros::Time last_request = ros::Time::now();
    ros::Time t_started_climbout = ros::Time::now();

    if(current_state.mode != "OFFBOARD"){
        if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
    }

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)) && _climbout_complete){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }

        double Dx =   current_pos.pose.position.x - true_pose.pose.position.x;
        double Dy =   current_pos.pose.position.y - true_pose.pose.position.y;
        double Dz =   current_pos.pose.position.z - true_pose.pose.position.z;

        if (Dz > 11.0f && _was_below){
          _is_climbout = true;
          _was_below = false;
        }

        if (Dz < 10.0f && !_was_below){
          _was_below = true;
        }

        if (_is_climbout){ //rate is 20hz --> this equals traveling away with at least 5m/s
          ROS_INFO("Climbing out");
          _climbout_complete = false; //don't switch into offboard

          wp_clear_srv.request = {};

          wp_push_srv.request.start_index = 0;
          wp_push_srv.request.waypoints.push_back(wp_msg);

          if (wp_clear_client.call(wp_clear_srv))
          {
              ROS_INFO("Waypoint list was cleared");
          }
          else
          {
              ROS_ERROR("Waypoint list couldn't been cleared");
          }

          if (wp_srv_client.call(wp_push_srv))
          {
            ROS_INFO("Success:%d", (bool)wp_push_srv.response.success);
            _waypoint_received = true;
          }
          else
          {
            ROS_ERROR("Waypoint couldn't be sent");

          }

          if (current_state.mode != "AUTO.MISSION" && _waypoint_received) {
            if( set_mode_client.call(auto_set_mode) && auto_set_mode.response.mode_sent){
              ROS_ERROR("AUTO.MISSION enabled");
              _is_climbout = false; //don't call this function anymore
              t_started_climbout = ros::Time::now();
              // ros::shutdown(); //optionally shut this node down
              // return 0;
            }
          }
        }
        if (ros::Time::now() - t_started_climbout > ros::Duration(40.0) && !_climbout_complete){// fly away for 40 seconds
          _climbout_complete = true;
        }



        if(add_noise and current_state.mode == "OFFBOARD"){
          //calculate the distance to the target
          double total_D = sqrt(Dx*Dx + Dy*Dy);
          // double x0;
          // double y0;
          // double z0;
          // if(Dz > 25.0f){
          //   z0 = 0.4 * (drand48() - 0.5); //average error of 1.6m
          //   target_pose.pose.position.z += z0; //(x)
          // }
          // if(total_D > 30.0f){
          //   x0 = 0.4 * (2/sqrt(2)) * (drand48() - 0.5);
          //   y0 = 0.4 * (2/sqrt(2)) * (drand48() - 0.5);
          //   target_pose.pose.position.x += x0; //(y)
          //   target_pose.pose.position.y += y0; //(x)
          //
          // } else {
          //   // x0 = std::min(0.008*total_D, 3.0) * (2/sqrt(2)) * (drand48() - 0.5); // calls every 0.05 seconds. with total_D=1 results in avg 4cm per second
          //   // y0 = std::min(0.008*total_D, 3.0) * (2/sqrt(2)) * (drand48() - 0.5);
          //   target_pose.pose.position.x = true_pose.pose.position.x; //(y)
          //   target_pose.pose.position.y = true_pose.pose.position.y; //(x)
          //   target_pose.pose.position.z = true_pose.pose.position.z;
          // }

          double x0 = std::min(0.004*total_D, 0.2) * (2/sqrt(2)) * (drand48() - 0.5); // calls every 0.05 seconds. with total_D=1 results in avg 4cm per second
          double y0 = std::min(0.004*total_D, 0.2) * (2/sqrt(2)) * (drand48() - 0.5);
          // double z0 = std::min(0.005*total_D, 0.5) * (2/sqrt(2)) * (drand48() - 0.5);

          target_pose.pose.position.x += x0; //(y)
          target_pose.pose.position.y += y0; //(x)
          // target_pose.pose.position.z += z0;

        }

        //pull back to original position:
        double dx =   target_pose.pose.position.x - true_pose.pose.position.x;
        double dy =   target_pose.pose.position.y - true_pose.pose.position.y;
        double dz =   target_pose.pose.position.z - true_pose.pose.position.z;
        //
        target_pose.pose.position.x -= 0.01*dx; //(y)
        target_pose.pose.position.y -= 0.01*dy; //(x)
        target_pose.pose.position.z -= 0.01*dz;

        local_pos_pub.publish(target_pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
