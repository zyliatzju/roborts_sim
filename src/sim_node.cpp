#include <string>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "roborts_msgs/GimbalAngle.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tfMessage.h"
#include "tf/transform_datatypes.h"

sensor_msgs::JointState gimbal;
//tf::StampedTransform tf_map_to_odom_;
//geometry_msgs::TransformStamped odom_trans;
float yawAngle = 0;
float pitchAngle = 0;
void gimbalInit()
{
    gimbal.header.stamp = ros::Time::now();
    gimbal.name.resize(2);
    gimbal.name[0] = "gimbalYaw";
    gimbal.name[1] = "gimbalPitch";
    gimbal.position.resize(2);
    gimbal.position[0] = 0.0;
    gimbal.position[1] = 0.0;
}
// void amclInit()
// {
//     odom_trans.header.stamp = ros::Time::now();
//     odom_trans.header.frame_id = "odom";
//     odom_trans.child_frame_id = "base_link";
//     odom_trans.transform.translation.x = 0;
//     odom_trans.transform.translation.y = 0;
//     odom_trans.transform.translation.z = 0;
//     tf::Quaternion quat_tf = tf::Quaternion(0, 0, 1, 0);
//     geometry_msgs::Quaternion quat_msg = geometry_msgs::Quaternion();
//     tf::quaternionTFToMsg(quat_tf, quat_msg);
//     odom_trans.transform.rotation = quat_msg;
// }

// void tfInit()
// {
//     tf_map_to_odom_.stamp_ = ros::Time::now();
//     // specify actual transformation vectors from odometry
//     // NOTE: zeros have to be substituted with actual variable data
//     tf_map_to_odom_.frame_id_ = "map";
//     tf_map_to_odom_.child_frame_id_ = "odom";
//     tf_map_to_odom_.setOrigin(tf::Vector3(0, 0, 0));
//     tf_map_to_odom_.setRotation(tf::Quaternion(0, 0, 1, 0));
// }

void gimbalAngleCallback(const roborts_msgs::GimbalAngle::ConstPtr &msg)
{
   yawAngle=msg->yaw_angle;
   pitchAngle=msg->pitch_angle;
}

// void amclCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {
//     odom_trans.header.frame_id = msg->header.frame_id; //"odom";
//     odom_trans.child_frame_id = "base_link";
//     odom_trans.header.stamp = msg->header.stamp; //ros::Time::now();
//     odom_trans.transform.translation.x = msg->pose.position.x;
//     odom_trans.transform.translation.y = msg->pose.position.y;
//     odom_trans.transform.translation.z = msg->pose.position.z;
//     odom_trans.transform.rotation = msg->pose.orientation;
// }

// void tfCallback(const tf::tfMessage::ConstPtr &msg)
// {
//     float a = msg->transforms.size();
//     ROS_INFO("(%f)",a);
//     tf_map_to_odom_.stamp_ = msg->transforms[0].header.stamp;
//     // specify actual transformation vectors from odometry
//     // NOTE: zeros have to be substituted with actual variable data
//     tf_map_to_odom_.frame_id_ = msg->transforms[0].header.frame_id;      // "map";
//     //std::cout << msg->transforms[0].header.frame_id << std::endl;
//     tf_map_to_odom_.child_frame_id_ = msg->transforms[0].child_frame_id; //"odom";
//     tf::Vector3 vec_tf = tf::Vector3();
//     tf::vector3MsgToTF(msg->transforms[0].transform.translation, vec_tf);
//     tf_map_to_odom_.setOrigin(vec_tf);
//     tf::Quaternion quat_tf = tf::Quaternion(0,0,1,0);
//    // tf::quaternionMsgToTF(msg->transforms[2].transform.rotation, quat_tf);
//     tf_map_to_odom_.setRotation(tf::Quaternion(
//         msg->transforms[0].transform.rotation.x,
//         msg->transforms[0].transform.rotation.y,
//         msg->transforms[0].transform.rotation.z,
//         msg->transforms[0].transform.rotation.w));
   
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim_node");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
    ros::Subscriber sub = n.subscribe("cmd_gimbal_angle", 10, gimbalAngleCallback);
   // ros::Subscriber sub1 = n.subscribe("amcl_pose", 10, amclCallback);
   // ros::Subscriber sub2 = n.subscribe("tf", 10, tfCallback);

    //tf::TransformBroadcaster broadcaster;
    //tf::TransformBroadcaster tf_br_;
    // set up parent and child frames
   // tfInit();

   // amclInit();

    gimbalInit();
    //pub.publish(gimbal);
    // robot state
    //double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;

    // message declarations
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        //update joint_state
        // float[] cmd=sub;

        //  std::cout<<p;

        // update transform

        //send the joint state and transform

        gimbal.header.stamp = ros::Time::now();
        gimbal.header.frame_id = "base_link";
    	gimbal.name.resize(2);
   		gimbal.position.resize(2);
    	gimbal.name[0] = "gimbalYaw";
    	gimbal.position[0] = yawAngle;
    	gimbal.name[1] = "gimbalPitch";
    	gimbal.position[1] = pitchAngle;
        pub.publish(gimbal);

       // tf_br_.sendTransform(tf_map_to_odom_);

      //  broadcaster.sendTransform(odom_trans);
        ros::spinOnce();
        // Create new robot state
        // This will adjust as needed per iteration
        loop_rate.sleep();
    }
    return 0;
}
