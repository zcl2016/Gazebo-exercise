#include <iostream>
#include <math.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/msgs.hh>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <boost/thread/mutex.hpp>

#define PI 3.14159265
#define V 0.8
#define RADIUS 5
#define NUMBER_OF_PUBLISHER 100

namespace gazebo
{

// geometry_msgs::Twist cmd_vel_;
// void cmd_velCommandCallback(const geometry_msgs::TwistConstPtr &command)
// {
// 	// std::cout << "Receive Topic" << std::endl;
// 	cmd_vel_ = *command;
// }

class TakeOff : public ModelPlugin
{
public:
	TakeOff(){};
	~TakeOff()
	{
		cmd_vel_subscriber_.shutdown();
		if (node_handle_) {
    		node_handle_->shutdown();
		}
		// gazebo::event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
		gazebo::event::EventAsyns::DisconnectWorldUpdateBegin(this->updateConnection);
	}

public:
	void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{
		this->model = _parent;
		world_ = model->GetWorld();
		// this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				// boost::bind(&TakeOff::OnUpdate, this, _1));
		this->updateConnection = event::EventAsyns::ConnectWorldUpdateBegin(
				boost::bind(&TakeOff::OnUpdate, this, _1));

		topicName = "cmd_vel";
		if (_sdf->HasElement("topicName"))    topicName= _sdf->GetElement("topicName")->Get<std::string>();
		robotNamespace = "TakeOff";
		if (_sdf->HasElement("robotNamespace")) robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>();

		// std::cout << "=====================TopicName: " << topicName.c_str() << "========================" << std::endl;
		// std::cout << "=====================RobotNamespace: " << robotNamespace.c_str() << "=====================" << std::endl;

		node_handle_ = new ros::NodeHandle("TakeOff");

		cmd_vel_subscriber_ = node_handle_->subscribe<geometry_msgs::Twist>(topicName, 1, boost::bind(
        &TakeOff::cmd_velCommandCallback, this, _1));
		// cmd_vel_subscriber_ = node_handle_->subscribe<geometry_msgs::Twist>(topicName, 1, cmd_velCommandCallback);
		for(int i = 0; i < NUMBER_OF_PUBLISHER; i++)
		{
			String_msg_publisher_[i] = node_handle_->advertise<std_msgs::String>("topic" + std::to_string(i),1000);
		}
			
		//this->o_pos = this->model->GetWorldPose();
	// 	node =  transport::NodePtr(new transport::Node());
	// 	node->Init(world_->GetName());
	// 	posePub = node->Advertise<msgs::PosesStamped>(
    // "~/pose/info", 10);
	}

public:
	void cmd_velCommandCallback(const geometry_msgs::TwistConstPtr &command)
	{
		// std::cout << "Receive Topic" << std::endl;
		cmd_vel_ = *command;
	}

public:
	void OnUpdate(const common::UpdateInfo & /*_info*/)
	{
		// 调用3到7的空域上下飞来飞去函数
		pos = this->model->GetWorldPose();
		this->stateControl(pos, 2, 80);
		//  msgs::PosesStamped msg;

      	// // Time stamp this PosesStamped message
      	// msgs::Set(msg.mutable_time(), world_->GetSimTime());
		// msgs::Pose *poseMsg = msg.add_pose();
		// std::cout << "Update the cmd_vel_" << std::endl;
		// ros::spinOnce();

		// 模拟发布话题topic
		for(int i = 0; i < NUMBER_OF_PUBLISHER; i++)
		{
			auto message = std_msgs::String();
    		message.data = "Hello, world! " + std::to_string(i);
			String_msg_publisher_[i].publish(message);
		}
		

        // Publish the model's relative pose
        // poseMsg->set_name(this->model->GetScopedName());
        // poseMsg->set_id(this->model->GetId());
        // msgs::Set(poseMsg, this->model->GetRelativePose().Ign());
		// posePub->Publish(msg);
	}

public:
	void stateControl(math::Pose &pos, double range_low, double range_high)
	{
		// caculate vertical vollcity
		if (pos.pos.z >= range_high)
		{
			this->vel_Z = -1;
		}
		else if (pos.pos.z <= range_low)
		{
			this->vel_Z = 1;
		}

		// usleep(3);
		// for(int x = 0; x < 320; x++)
		// {
		// 	for(int y = 0; y < 100; y++)
		// 	{

		// 		int z = 9;
		// 		z /= 5 + 6 - 9 * 10;
		// 	}
		// }
		// double x = 0.0;
		// double y = 0.0;
		// double z = 2.0;
		// double R = 0.0;
		// double P = 0.3;
		// double Y = 0.0;
		// math::Pose pose(x,y,z,R,P,Y);
		// math::Pose pos = link_->GetWorldCoGPose();
		//pose.pos.x = 0.0;
		//pose.pos.y = 0.0;
		//pose.pos.z = 2.0;
		// pos.rot.x = msg.roll_angle * M_PI / 180.0;
		// pos.rot.y = -msg.pitch_angle * M_PI / 180.0;
		// pos.rot.z = -msg.yaw_angle * M_PI / 180.0;
		//pose.rot.x = 0.0;
		//pose.rot.y = 0.3;
		//pose.rot.z = 0.0;

		// this->model->SetLinearVel(ignition::math::Vector3d(0, 0, this->vel_Z));
		// this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0.5));
		// this->model->SetWorldPose(pose);
		this->model->SetLinearVel(ignition::math::Vector3d(cmd_vel_.linear.x, cmd_vel_.linear.y, cmd_vel_.linear.z));
		this->model->SetAngularVel(ignition::math::Vector3d(cmd_vel_.angular.x, cmd_vel_.angular.y, cmd_vel_.angular.z));
	}

private:
	physics::ModelPtr model;

private:
	event::ConnectionPtr updateConnection;

private:
	math::Pose pos;

private:
	math::Pose o_pos;

private:
	double vel_Z = 0;

private:
	double vel_X = 0;

private:
	double vel_Y = 0;

private:
	double theta;

public: 
	transport::PublisherPtr posePub;

public: 
	transport::NodePtr node;
public:
	gazebo::physics::WorldPtr world_;
	ros::Subscriber cmd_vel_subscriber_;

	ros::Publisher String_msg_publisher_[NUMBER_OF_PUBLISHER];
	geometry_msgs::Twist cmd_vel_;
	ros::NodeHandle* node_handle_;
	std::string topicName;
	std::string robotNamespace;

};
GZ_REGISTER_MODEL_PLUGIN(TakeOff)
} // namespace gazebo
