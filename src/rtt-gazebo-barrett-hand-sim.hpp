#pragma once

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/Semaphore.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>

#include <Eigen/Dense>
#include <Eigen/LU>

#include <vector>

#include <Eigen/Core>
#include <time.h>
#include <rtt/os/TimeService.hpp>
#include <sstream>
#include <rtt/Logger.hpp>

#include <thread>
#include <memory>
#include <string>
#include <fstream>
#include <streambuf>
#include <sstream>

#include <control_modes.h>
#include <boost/shared_ptr.hpp>

#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>

#include <kdl/velocityprofile_trap.hpp>

namespace cosima
{

class BarrettHandSim : public RTT::TaskContext
{
  public:
	BarrettHandSim(std::string const &name);
	bool configureHook();
	void updateHook();
	void WorldUpdateBegin();
	void WorldUpdateEnd();
	// virtual ~BarrettHandSim()
	// {
	// }

  protected:
	bool getModel(const std::string &model_name);
	void gazeboUpdateHook(gazebo::physics::ModelPtr model);
	bool gazeboConfigureHook(gazebo::physics::ModelPtr model);

	bool registerSensors();

	int N_PUCKS;

	// void setupPorts();
	// void setupVars();

	bool
	setControlMode(std::string controlMode);

	// void getLaserData();
	// void processLaserData(gazebo::sensors::RaySensorPtr sensor, gazebo::physics::MultiRayShapePtr rayShape);

	gazebo::physics::ModelPtr model;
	gazebo::event::ConnectionPtr world_begin;
	gazebo::event::ConnectionPtr world_end;

	RTT::SendHandle<gazebo::physics::ModelPtr(const std::string &, double)> get_model_handle;

	gazebo::physics::Joint_V model_joints_;
	gazebo::physics::Link_V model_links_;

	std::vector<int> joints_idx_;
	std::vector<std::string> joint_names_;

	//    Ports:
	RTT::OutputPort<rstrt::robot::JointState> out_hand_JointFeedback_port;
	rstrt::robot::JointState out_hand_JointFeedback;

	RTT::InputPort<rstrt::kinematics::JointAngles> in_hand_JointPositionCtrl_port;
	RTT::FlowStatus in_hand_JointPositionCtrl_flow;
	rstrt::kinematics::JointAngles in_hand_JointPositionCtrl;
	rstrt::kinematics::JointAngles in_hand_JointPositionCtrl_new;

	RTT::InputPort<rstrt::kinematics::JointVelocities> in_hand_JointVelocityCtrl_port;
	RTT::FlowStatus in_hand_JointVelocityCtrl_flow;
	rstrt::kinematics::JointVelocities in_hand_JointVelocityCtrl;
	rstrt::kinematics::JointVelocities in_hand_JointVelocityCtrl_new;

	RTT::InputPort<rstrt::dynamics::JointTorques> in_hand_JointTorqueCtrl_port;
	RTT::FlowStatus in_hand_JointTorqueCtrl_flow;
	rstrt::dynamics::JointTorques in_hand_JointTorqueCtrl;
	rstrt::dynamics::JointTorques in_hand_JointTorqueCtrl_new;

	// int n_allJoints;

	void initialize();
	void idle();
	void run();

	void setCompliance(bool enable);

	// void setTorqueMode(unsigned int joint_index);
	// void setPositionMode(unsigned int joint_index);
	// void setVelocityMode(unsigned int joint_index);
	// void setTrapezoidalMode(unsigned int joint_index);
	// void setIdleMode(unsigned int joint_index);

	void readSim();
	void writeSim();

	void writeFeedbackToOrocos();
	void readCommandsFromOrocos();

	const bool fingerToJointIDs(const uint finger_id, uint &medial_id, uint &distal_id);

	void open();
	void close();
	void openSpread();
	void closeSpread();

	bool
	doneMoving(const unsigned pair_index);
	bool withinTorqueLimits(const unsigned joint_index);

	// std::vector<gazebo::physics::JointPtr> gazebo_joints;

	bool compliance_enabled;

	double
		breakaway_torque,
		stop_torque;

	Eigen::VectorXd
		link_torque,
		fingertip_torque,
		breakaway_angle,
		joint_torque,
		joint_torque_max,
		joint_torque_breakaway;

	std::vector<KDL::VelocityProfile_Trap> trap_generators;
	// std::vector<ros::Time> trap_start_times;
	std::vector<bool> torque_switches;

	double
		p_gain,
		d_gain,
		velocity_gain;

	enum RunMode
	{
		IDLE,
		INITIALIZE,
		RUN
	};

	enum InitState
	{
		INIT_FINGERS,
		SEEK_FINGERS,
		SEEK_SPREAD,
		INIT_CLOSE
	};

	RunMode run_mode;
	InitState init_state;

  private:
	bool is_configured;

	std::string currentControlMode;

	// //Laser Stuff
	// gazebo::sensors::RaySensorPtr frontSensor;
	// gazebo::sensors::RaySensorPtr backSensor;
	// bool sensorsRegistered;

	std::string urdf_prefix;
};

} // namespace cosima