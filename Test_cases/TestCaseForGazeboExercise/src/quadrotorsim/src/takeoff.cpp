/************************************************************
 * Author: zenglei                                          *
 * Date:  2019-12-03                                        *
 * Description: the modelplugin of takeoff for UAV          *
 * Modified: None                                           *
 * **********************************************************/

#include <iostream>
#include <math.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/msgs.hh>

#define PI 3.14159265
#define V 0.8
#define RADIUS 5

namespace gazebo
{
class TakeOff : public ModelPlugin
{
public:
	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
	{
		this->model = _parent;
		world_ = model->GetWorld();
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&TakeOff::OnUpdate, this, _1));
		//this->o_pos = this->model->GetWorldPose();
	// 	node =  transport::NodePtr(new transport::Node());
	// 	node->Init(world_->GetName());
	// 	posePub = node->Advertise<msgs::PosesStamped>(
    // "~/pose/info", 10);
	}

public:
	void OnUpdate(const common::UpdateInfo & /*_info*/)
	{
		// 调用3到7的空域上下飞来飞去函数
		pos = this->model->GetWorldPose();
		this->stateControl(pos, 3, 7);
		 msgs::PosesStamped msg;

      	// Time stamp this PosesStamped message
      	msgs::Set(msg.mutable_time(), world_->GetSimTime());
		msgs::Pose *poseMsg = msg.add_pose();

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
		// if (pos.pos.z >= range_high)
		// {
		// 	this->vel_Z = -1;
		// }
		// else if (pos.pos.z <= range_low)
		{
			this->vel_Z = 1;
		}
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

		this->model->SetLinearVel(ignition::math::Vector3d(0, 0, this->vel_Z));
		this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0.5));
		// this->model->SetWorldPose(pose);
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

};
GZ_REGISTER_MODEL_PLUGIN(TakeOff)
} // namespace gazebo