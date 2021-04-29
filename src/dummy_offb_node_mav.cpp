/**
 * @file dummy_offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <stdlib.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mav_msgs/RollPitchYawrateThrust.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "dummy_offb_node_mav");
    ros::NodeHandle nh;


    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Publisher rpyt_pub = nh.advertise<mav_msgs::RollPitchYawrateThrust>
          ("mavros/setpoint_raw/roll_pitch_yawrate_thrust", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
     ("mavros/cmd/arming");

    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
     ("mavros/cmd/takeoff");

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
    mav_msgs::RollPitchYawrateThrust rpyt_msg;

    // msg.header.stamp = ros::Time::now();
    // msg.header.frame_id = "map";
    rpyt_msg.roll = 0.0f;
    rpyt_msg.pitch = 0.0f;
    rpyt_msg.yaw_rate = 0.0f;
    rpyt_msg.thrust.z = 5000; //TODO adjust thrust_scaling in drogone_startup/parameters/mavros/px4_config.yaml

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::SetMode position_set_mode;
    position_set_mode.request.custom_mode = "POSCTL";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.altitude = 5; //might be AMSL like 510 or something along the lines

    // //send a few setpoints before starting
    // for(int i = 10; ros::ok() && i > 0; --i){
    //     rpyt_pub.publish(rpyt_msg);
    //     ros::spinOnce();
    //     rate.sleep();
    // }


    ros::Time last_request = ros::Time::now();
    //
    //
    // if( !current_state.armed){
    //     if( arming_client.call(arm_cmd) &&
    //         arm_cmd.response.success){
    //         ROS_INFO("Vehicle armed");
    //     }
    // }
    bool in_air = false;

    if (current_state.mode != "AUTO.POSITION"){
      if( set_mode_client.call(position_set_mode) &&
          position_set_mode.response.mode_sent){
          ROS_INFO("Position enabled");
      }
    }

    while(ros::ok()){

      //check if we are already in air, only works if we takeoff using this node
      //after auto.takeoff it switches to auto.loiter mode by default
      if(current_state.mode == "AUTO.LOITER") {
        in_air = true;
      }

      //switch to takeoff state
      if( !in_air && ros::Time::now() - last_request > ros::Duration(2.0)){
        if (current_state.mode != "AUTO.TAKEOFF"){
          if(takeoff_client.call(takeoff_cmd) && takeoff_cmd.response.success){
            ROS_INFO("Takeoff Mode");
          }
        }
        if (!current_state.armed){
          if( arming_client.call(arm_cmd) &&
              arm_cmd.response.success){
              ROS_INFO("Vehicle armed");
          }
        }
        last_request = ros::Time::now();

      }


      if( in_air && current_state.mode != "OFFBOARD" && ros::Time::now() - last_request > ros::Duration(5.0))
      {
          if( set_mode_client.call(offb_set_mode) &&
              offb_set_mode.response.mode_sent){
              ROS_INFO("Offboard enabled");
          }
          last_request = ros::Time::now();
      }

      rpyt_pub.publish(rpyt_msg);

      ros::spinOnce();
      rate.sleep();
    }

    return 0;
}
