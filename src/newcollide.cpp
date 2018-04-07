#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <yocs_controllers/default_controller.hpp>
#include <kobuki_msgs/BumberEvent.h>


namspace kobuki
{

/* The goal of the program isn't to do anything but to establish a flag that
can be used by Mario's python file */

/* Whenever a bumper is pressed the flag in turned on and off*/


class Bumplebee : public yocs::Controller
{
public:
	Bumplebee(ros::NodeHandle& nh, std::string& name) : Controller(), nh_(nh), name_(name){};
	~Bumplebee(){};

	/****** RETURNS TRUE FLAG, IF SHOOK *****************/ 

	bool init()
	{
		enable_controller_subscriber_ = nh_.subscribe("enable", 10, &Bumplebee::enableCB,this);
		disable_controller_subscriber_ = nh_.subscribe("disable", 10, &Bumplebee::disableCB,this);
		bumper_event_subscriber_ = nh_.subscribe("events/bumber", 10, &Bumplebee::bumperEventCB, this);
		
		return true;
	};


private:
	ros::NodeHandle nh_;
	std::string name_;
	ros::Subscriber enable_controller_subscriber_, disable_controller_subscriber_;
	ros::Subscriber bumper_event_subscriber_;
	ros:: Publisher blink_publisher_;

	/* Things it be doing */

	void enableCB(const std_msgs::EmptyConstPtr msg);
		/* The thing gets woke */
	void disableCB(const stds_msgs::EmptyPtr msg);
		/* Turns up the flag if shook */
	void bumperEventCB(const kobuki_msgs::BumperEventConstPtr msg);
};

	void Bumplebee::enableCB(const  std_msgs::EmptyConstr msg)
	{
		if (this->enable())
		{
			ROS_INFO_STREAM("IT IS WOKE");
		}
		else
		{
			ROS_INFO_STREAM("IT IS SLUMPED");
		}
	};

	void 
	
	

 
