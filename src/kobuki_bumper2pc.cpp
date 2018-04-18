/**
 * @file /src/kobuki_bumper2pc.cpp
 *
 * @brief Bumper to pointcloud nodelet class implementation.
 *
 * @author Jorge Santos, Yujin Robot
 *
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <pluginlib/class_list_macros.h>

#include "kobuki_bumper2pc/kobuki_bumper2pc.hpp"

#include <std_msgs/Int32.h>
namespace kobuki_bumper2pc
{

void Bumper2PcNodelet::coreSensorCB(const kobuki_msgs::SensorState::ConstPtr& msg)
{
	int flag;
  if (pointcloud_pub_.getNumSubscribers() == 0)
    return;

  // We publish just one "no events" pc (with all three points far away) and stop spamming when bumper/cliff conditions disappear
  if (! msg->bumper && ! msg->cliff && ! prev_bumper && ! prev_cliff)
    return;

  prev_bumper = msg->bumper;
  prev_cliff  = msg->cliff;

  // We replicate the sensors order of bumper/cliff event messages: LEFT = 0, CENTER = 1 and RIGHT = 2
  // For any of {left/center/right} with no bumper/cliff event, we publish a faraway point that won't get used 
  if ((msg->bumper & kobuki_msgs::SensorState::BUMPER_LEFT) ||
      (msg->cliff  & kobuki_msgs::SensorState::CLIFF_LEFT))
  {
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[0].offset], &p_side_x_, sizeof(float));
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[1].offset], &p_side_y_, sizeof(float));
    /* FLAG GOES HERE*/
	flag=1;
  }
  else
  {
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[1].offset], &P_INF_Y, sizeof(float));
  	 flag=0;
  }

  if ((msg->bumper & kobuki_msgs::SensorState::BUMPER_CENTRE) ||
      (msg->cliff  & kobuki_msgs::SensorState::CLIFF_CENTRE))
  {
    memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &pc_radius_, sizeof(float));
	flag=2;
 }
  else
  {
    memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
	flag=0;
  }

  if ((msg->bumper & kobuki_msgs::SensorState::BUMPER_RIGHT) ||
      (msg->cliff  & kobuki_msgs::SensorState::CLIFF_RIGHT))
  {
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[0].offset], &p_side_x_, sizeof(float));
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[1].offset], &n_side_y_, sizeof(float));
	flag=1;
	}
  else
  {
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[1].offset], &N_INF_Y, sizeof(float));
	flag=0;  
}

	pointcloud_.header.stamp = msg->header.stamp;
	pointcloud_pub_.publish(pointcloud_);
	std_msgs::Int32 msg2;
	msg2.data=flag;
	collide_pub.publish(msg2);  
}

void Bumper2PcNodelet::onInit()
{
  ros::NodeHandle nh = this->getPrivateNodeHandle();

  // Bumper/cliff pointcloud distance to base frame; should be something like the robot radius plus
  // costmap resolution plus an extra to cope with robot inertia. This is a bit tricky parameter: if
  // it's too low, costmap will ignore this pointcloud (the robot footprint runs over the hit obstacle),
  // but if it's too big, hit obstacles will be mapped too far from the robot and the navigation around
  // them will probably fail.
  std::string base_link_frame;
  double r, h, angle;
  nh.param("pointcloud_radius", r, 0.175); pc_radius_ = r;
  nh.param("pointcloud_height", h, 0.094); pc_height_ = h;
  nh.param("side_point_angle", angle, 6.0734906585); 
  nh.param<std::string>("base_link_frame", base_link_frame, "/base_link");

  // Lateral points x/y coordinates; we need to store float values to memcopy later
  p_side_x_ = + pc_radius_*sin(angle); // angle degrees from vertical
  p_side_y_ = + pc_radius_*cos(angle); // angle degrees from vertical
  n_side_y_ = - pc_radius_*cos(angle); // angle degrees from vertical

  // Prepare constant parts of the pointcloud message to be  published
  pointcloud_.header.frame_id = base_link_frame;
  pointcloud_.width  = 2;
  pointcloud_.height = 1;
  pointcloud_.fields.resize(3);

  // Set x/y/z as the only fields
  pointcloud_.fields[0].name = "x";
  pointcloud_.fields[1].name = "y";
  pointcloud_.fields[2].name = "z";

  int offset = 0;
  // All offsets are *4, as all field data types are float32
  for (size_t d = 0; d < pointcloud_.fields.size(); ++d, offset += 4)
  {
    pointcloud_.fields[d].count    = 1;
    pointcloud_.fields[d].offset   = offset;
    pointcloud_.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
  }

  pointcloud_.point_step = offset;
  pointcloud_.row_step   = pointcloud_.point_step * pointcloud_.width;

  pointcloud_.data.resize(3 * pointcloud_.point_step);
  pointcloud_.is_bigendian = false;
  pointcloud_.is_dense     = true;

  // Bumper/cliff "points" fix coordinates (the others depend on sensor activation/deactivation)

  // y: always 0 for central bumper
  memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[1].offset], &ZERO, sizeof(float));

  // z: constant elevation from base frame
  memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));
  memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));
  memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));

	pointcloud_pub_  = nh.advertise <sensor_msgs::PointCloud2> ("pointcloud", 10);
	core_sensor_sub_ = nh.subscribe("core_sensors", 10, &Bumper2PcNodelet::coreSensorCB, this);

	ROS_INFO("Bumper/cliff pointcloud configured at distance %f and height %f from base frame", pc_radius_, pc_height_);
	ros::Publisher collide_pub = nh.advertise<std_msgs::Int32>("Flag",10);
}

} // namespace kobuki_bumper2pc


PLUGINLIB_EXPORT_CLASS(kobuki_bumper2pc::Bumper2PcNodelet, nodelet::Nodelet);
