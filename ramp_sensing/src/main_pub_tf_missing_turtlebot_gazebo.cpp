#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

/*
 * Create broadcasters
 * br_parent_child
 * bf   = base_footprint
 * bl   = base_link
 * cl   = camera_link
 * crf  = camera_rgb_frame
 * cdf  = camera_depth_frame
 * cdof = camera_depth_optimal_frame
 * crof = camera_rgb_optical_frame
 */

// Create transforms
tf::Transform tf_bf_bl, tf_bl_crf, tf_crf_cdf, tf_cdf_cdof, tf_crf_cl, tf_crf_crof;


void set_tf_bf_bl(tf::Transform& tf)
{
  // Set position vector
  tf.setOrigin( tf::Vector3(0, 0, 0.0102) );

  // Set rotation
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  tf.setRotation(q);
}

// parent: base_link child: camera_rgb_frame
void set_tf_bl_crf(tf::Transform& tf)
{
  // Set position vector
  tf.setOrigin( tf::Vector3(0.087, 0.013, -0.287) );

  // Set rotation
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  tf.setRotation(q);
}

// parent: camera_rgb_frame child: camera_depth_frame
void set_tf_crf_cdf(tf::Transform& tf)
{
  // Set position vector
  tf.setOrigin( tf::Vector3(0, 0, 0) );

  // Set rotation
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  tf.setRotation(q);
}

// parent: camera_depth_frame child: camera_depth_optical_frame
void set_tf_cdf_cdof(tf::Transform& tf)
{
  // Set position vector
  tf.setOrigin( tf::Vector3(0, 0, 0) );

  // Set rotation
  tf::Quaternion q;
  q.setRPY(1.5708, -1.5708, 0);
  tf.setRotation(q);
}

// parent: camera_rgb_frame child: camera_link
void set_tf_crf_cl(tf::Transform& tf)
{
  // Set position vector
  tf.setOrigin( tf::Vector3(0, 0.022, 0) );

  // Set rotation
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  tf.setRotation(q);
}

// parent: camera_rgb_frame child: camera_rgb_optical_frame
void set_tf_crf_crof(tf::Transform& tf)
{
  // Set position vector
  tf.setOrigin( tf::Vector3(0, 0, 0) );

  // Set rotation
  tf::Quaternion q;
  q.setRPY(1.5708, -1.5708, 0);
  tf.setRotation(q);
}

void setTFs()
{
  set_tf_bf_bl    (tf_bf_bl   );
  set_tf_bl_crf   (tf_bl_crf  );
  set_tf_crf_cdf  (tf_crf_cdf );
  set_tf_cdf_cdof (tf_cdf_cdof);
  set_tf_crf_cl   (tf_crf_cl  );
  set_tf_crf_crof (tf_crf_crof);
}


void sendTF(const ros::TimerEvent& e)
{
  /*
   * Create broadcasters
   * br_parent_child
   * bf   = base_footprint
   * bl   = base_link
   * cl   = camera_link
   * crf  = camera_rgb_frame
   * cdf  = camera_depth_frame
   * cdof = camera_depth_optimal_frame
   * crof = camera_rgb_optical_frame
   */
  static tf::TransformBroadcaster br_bf_bl, br_bl_crf, br_crf_cdf, br_cdf_cdof, br_crf_cl, br_crf_crof;

  /*
   * Broadcast transforms
   * sendTransform( StampedTransform, time(now+0.5), "parent", "child")
   */
  br_bf_bl.sendTransform(tf::StampedTransform(tf_bf_bl, ros::Time::now()+ros::Duration(0.05), "base_footprint", "base_link"));
  br_bl_crf.sendTransform(tf::StampedTransform(tf_bl_crf, ros::Time::now()+ros::Duration(0.05), "base_link", "camera_rgb_frame"));
  br_crf_cdf.sendTransform(tf::StampedTransform(tf_crf_cdf, ros::Time::now()+ros::Duration(0.05), "camera_rgb_frame", "camera_depth_frame"));
  br_cdf_cdof.sendTransform(tf::StampedTransform(tf_cdf_cdof, ros::Time::now()+ros::Duration(0.05), "camera_depth_frame", "camera_depth_optical_frame"));
  br_crf_cl.sendTransform(tf::StampedTransform(tf_crf_cl, ros::Time::now()+ros::Duration(0.05), "camera_rgb_frame", "camera_link"));
  br_crf_crof.sendTransform(tf::StampedTransform(tf_crf_crof, ros::Time::now()+ros::Duration(0.05), "camera_rgb_frame", "camera_rgb_optical_frame"));
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "pub_tf_missing_turtlebot_gazebo");
  ros::NodeHandle handle;

  // Set transforms
  setTFs();

  // Create Timer to publish transforms at 20Hz
  ros::Timer tf_timer = handle.createTimer(ros::Duration(0.05), &sendTF);

  ROS_INFO("Timer created");
  ROS_INFO("Publishing missing transforms");

  ros::spin();
  return 0;
}
