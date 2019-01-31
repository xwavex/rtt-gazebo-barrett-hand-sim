#include "rtt-gazebo-barrett-hand-sim.hpp"
#include <rtt/Operation.hpp>
#include <string>
#include <fstream>
#include <streambuf>
#include <sstream>
#include <control_modes.h>

// #define N_PUCKS 4;

using namespace cosima;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

BarrettHandSim::BarrettHandSim(const std::string &name) : TaskContext(name), is_configured(false), N_PUCKS(4), urdf_prefix(""),
														  compliance_enabled(false),
														  breakaway_torque(2.5),
														  stop_torque(3),
														  link_torque(4),
														  fingertip_torque(4),
														  breakaway_angle(4),
														  joint_torque(8),
														  joint_torque_max(Eigen::VectorXd::Constant(8, 1.5)),
														  joint_torque_breakaway(4),
														  p_gain(25.0),
														  d_gain(1.0),
														  velocity_gain(0.1),
														  torque_switches(4, false)
//  , n_allJoints(8)
{
	this->provides("gazebo")->addOperation("WorldUpdateBegin",
										   &BarrettHandSim::WorldUpdateBegin, this, RTT::ClientThread);
	this->provides("gazebo")->addOperation("WorldUpdateEnd",
										   &BarrettHandSim::WorldUpdateEnd, this, RTT::ClientThread);

	this->addOperation("initialize", &BarrettHandSim::initialize, this, ClientThread);

	this->addOperation("getModel", &BarrettHandSim::getModel, this, ClientThread);

	this->addOperation("setControlMode", &BarrettHandSim::setControlMode, this, ClientThread);

	this->addOperation("fixAngles", &BarrettHandSim::fixAngles, this, ClientThread);

	this->addOperation("open", &BarrettHandSim::open, this, ClientThread);
	this->addOperation("close", &BarrettHandSim::close, this, ClientThread);
	this->addOperation("openSpread", &BarrettHandSim::openSpread, this, ClientThread);
	this->addOperation("closeSpread", &BarrettHandSim::closeSpread, this, ClientThread);

	this->addProperty("currentControlMode", currentControlMode);

	this->addProperty("stop_torque", stop_torque);
	this->addProperty("breakaway_torque", breakaway_torque);

	this->addProperty("p_gain", p_gain);
	this->addProperty("d_gain", d_gain);

	this->addProperty("velocity_gain", velocity_gain);

	this->addProperty("urdf_prefix", urdf_prefix);

	s_angle = 0;
	f_angle = 0;
	b_angle = 0;
	this->addProperty("s_angle", s_angle);
	this->addProperty("f_angle", f_angle);
	this->addProperty("b_angle", b_angle);

	// this->addOperation("debugVelocity", &BarrettHandSim::debugVelocity, this, ClientThread);
	// this->addOperation("debugPose", &BarrettHandSim::debugPose, this, ClientThread);
	// this->addOperation("debugRelativePose", &BarrettHandSim::debugRelativePose, this, ClientThread);
	// this->addOperation("debugComputeMainVelocity", &BarrettHandSim::debugComputeMainVelocity, this, ClientThread);
	// this->addOperation("registerSensors", &BarrettHandSim::registerSensors, this, ClientThread);
	// this->addOperation("setForceFieldVars", &BarrettHandSim::setForceFieldVars, this, ClientThread);

	// this->addPort("in_velocity_cmd_port", in_velocity_cmd_port).doc("Main velocity input");
	// this->addPort("in_pose_cmd_port", in_pose_cmd_port).doc("Absolute position input");
	// this->addPort("in_relativePose_cmd_port", in_relativePose_cmd_port).doc("Relative position input");
	// this->addPort("out_pose_fdb_port", out_pose_fdb_port).doc("Port for position feedback");
	// this->addPort("out_desiredPose_port", out_desiredPose_port.doc("Port for desired position feedback"));
	// this->addPort("out_velocity_fdb_port", out_velocity_fdb_port).doc("Port for velocity feedback");
	// this->addPort("out_desiredTwist_port", out_desiredTwist_port).doc("Port for desired twist feedback");
	// this->addPort("out_controllerUpdate_fdb_port", out_controllerUpdate_fdb_port).doc("Port for computed controller update");
	// this->addPort("out_obstacle_port", out_obstacle_port);
	// this->addPort("out_force_fdb_port", out_force_fdb_port);

	// this->addPort("out_controllerDebug_port", out_controllerDebug_port).doc("Port for controller debug output");

	setControlMode(ControlModes::VelocityCtrl);
	idle();

	world_begin = gazebo::event::Events::ConnectWorldUpdateBegin(
		boost::bind(&BarrettHandSim::WorldUpdateBegin, this));
	world_end = gazebo::event::Events::ConnectWorldUpdateEnd(
		boost::bind(&BarrettHandSim::WorldUpdateEnd, this));
}

bool BarrettHandSim::getModel(const std::string &model_name)
{
	if (model)
	{
		log(Warning) << "Model [" << model_name << "] already loaded !"
					 << endlog();
		return true;
	}
	gazebo::printVersion();
	if (!gazebo::physics::get_world())
	{
		log(Error) << "getWorldPtr does not seem to exists" << endlog();
		return false;
	}
	model = gazebo::physics::get_world()->GetModel(model_name);
	if (model)
	{
		log(Info) << "Model [" << model_name << "] successfully loaded !"
				  << endlog();
		return true;
	}
	return bool(model);
}

void BarrettHandSim::updateHook()
{
}

bool BarrettHandSim::configureHook()
{
	// setupPorts();
	// setupVars();

	this->is_configured = gazeboConfigureHook(model);
	// this->initialize();

	out_hand_JointFeedback = rstrt::robot::JointState(model_joints_.size());
	this->addPort("hand_JointFeedback", out_hand_JointFeedback_port).doc("Output port for the joint feedback.");
	// rstrt::robot::JointState out_hand_JointFeedback = rstrt::robot::JointState(model_joints_.size());
	out_hand_JointFeedback_port.setDataSample(out_hand_JointFeedback);

	this->addPort("hand_JointPositionCtrl", in_hand_JointPositionCtrl_port).doc("Input port for commands based on position control mode");
	in_hand_JointPositionCtrl_flow = RTT::NoData;
	in_hand_JointPositionCtrl = rstrt::kinematics::JointAngles(N_PUCKS);
	in_hand_JointPositionCtrl_new = rstrt::kinematics::JointAngles(N_PUCKS);

	this->addPort("hand_JointVelocityCtrl", in_hand_JointVelocityCtrl_port).doc("Input port for commands based on velocity control mode");
	in_hand_JointVelocityCtrl_flow = RTT::NoData;
	in_hand_JointVelocityCtrl = rstrt::kinematics::JointVelocities(N_PUCKS);
	in_hand_JointVelocityCtrl_new = rstrt::kinematics::JointVelocities(N_PUCKS);

	this->addPort("hand_JointTorqueCtrl", in_hand_JointTorqueCtrl_port).doc("Input port for commands based on torque control mode");
	in_hand_JointTorqueCtrl_flow = RTT::NoData;
	in_hand_JointTorqueCtrl = rstrt::dynamics::JointTorques(N_PUCKS);
	in_hand_JointTorqueCtrl_new = rstrt::dynamics::JointTorques(N_PUCKS);

	if (parentJointForFT)
	{
		out_hand_FT = rstrt::dynamics::Wrench();
		this->addPort("hand_FT", out_hand_FT_port).doc("Output port for the force torque feedback.");
		out_hand_FT_port.setDataSample(out_hand_FT);
	}

	return is_configured;
}

// void BarrettHandSim::setupPorts()
// {
// in_pose_cmd_flow = RTT::NoData;
// in_pose_cmd = rstrt::geometry::Pose(rstrt::geometry::Translation(0, 0, 0), rstrt::geometry::Rotation(0, 0, 0, 0));

// in_relativePose_cmd_flow = RTT::NoData;
// in_relativePose_cmd = rstrt::geometry::Pose(rstrt::geometry::Translation(0, 0, 0), rstrt::geometry::Rotation(0, 0, 0, 0));

// in_velocity_cmd_flow = RTT::NoData;
// in_velocity_cmd = rstrt::kinematics::Twist(0, 0, 0, 0, 0, 0);

// out_velocity_fdb = rstrt::kinematics::Twist(0, 0, 0, 0, 0, 0);
// out_velocity_fdb_port.setDataSample(out_velocity_fdb);

// out_pose_fdb = rstrt::geometry::Pose();
// out_pose_fdb_port.setDataSample(out_pose_fdb);

// out_desiredPose_fdb = rstrt::geometry::Translation(0, 0, 0);
// out_desiredPose_port.setDataSample(out_desiredPose_fdb);

// out_desiredTwist_fdb = rstrt::geometry::Translation(0, 0, 0);
// out_desiredTwist_port.setDataSample(out_desiredTwist_fdb);

// out_controllerUpdate_fdb = rstrt::geometry::Translation(0, 0, 0);
// out_controllerUpdate_fdb_port.setDataSample(out_controllerUpdate_fdb);

// out_controllerDebug_fdb = "";
// out_controllerDebug_port.setDataSample(out_controllerDebug_fdb);

// out_obstacle_fdb = rstrt::geometry::Translation(0, 0, 0);
// out_obstacle_port.setDataSample(out_obstacle_fdb);

// out_force_fdb = rstrt::kinematics::Twist(0, 0, 0, 0, 0, 0);
// out_force_fdb_port.setDataSample(out_force_fdb);
// }

// void BarrettHandSim::setupVars()
// {
// tmp = Eigen::VectorXd(5);

// sensorsRegistered = false;
// debugAlpha = 0.0;
// maxVelocity = 0.8;
// maxAcceleration = 1.4;
// omnirobRadius = 0.602;
// timestep = 0.001;

// desiredTwist = Eigen::Vector3d::Zero();
// newTwist = Eigen::Vector3d::Zero();
// lastVelError = Eigen::Vector3d::Zero();
// velocityVector = Eigen::Vector3d::Zero();

// desiredPose = Eigen::Vector3d::Zero();
// newPose = Eigen::Vector3d::Zero();
// actualPoseError = Eigen::Vector3d::Zero();
// lastPoseError = Eigen::Vector3d::Zero();

// setControlMode("VelocityCtrl");

// jacobian.resize(4, 3);
// jacobian << 1, -1, -(0.347 + 0.24), 1, 1, (0.347 + 0.24), 1, 1, -(0.347 + 0.24), 1, -1, (0.347 + 0.24);

// pseudoInverse.resize(4, 3);
// pseudoInverse = (jacobian.transpose() * jacobian).inverse() * jacobian.transpose();

// radia.resize(4);
// radia << 0.125, 0.125, 0.125, 0.125;

// //Force Field Section
// force_factor = 5;
// dMax_factor = 2;
// dMin_factor = 0.2;
// vMax_factor = 2;

// resultVector = Eigen::Vector3d::Zero();
// }

bool BarrettHandSim::gazeboConfigureHook(gazebo::physics::ModelPtr model)
{
	if (model.get() == NULL)
	{
		RTT::log(RTT::Error) << "Model could not be loaded!" << RTT::endlog();
		return false;
	}
	// Get the joints
	// model_joints_ = model->GetJoints();
	// model_links_ = model->GetLinks();

	joints_idx_.clear();
	joint_names_.clear();
	model_joints_.clear();

	gazebo::physics::LinkPtr firstLink = model->GetLink(urdf_prefix + "bhand_palm_link");
	if (firstLink)
	{
		if (firstLink->GetParentJoints().size() == 1)
		{
			parentJointForFT = firstLink->GetParentJoints()[0];
			if (parentJointForFT)
			{
				RTT::log(RTT::Info) << "Force Torque Sensor found!" << RTT::endlog();
			}
			else
			{
				RTT::log(RTT::Error) << "Force Torque Sensor not found! First parent joint ist empty!" << RTT::endlog();
			}
		}
		else
		{
			RTT::log(RTT::Error) << "Force Torque Sensor not found! No parent joints at all!" << RTT::endlog();
		}
	}
	else
	{
		RTT::log(RTT::Error) << "Force Torque Sensor not found! Link " << urdf_prefix << "bhand_palm_link not found!" << RTT::endlog();
	}

	gazebo::physics::JointPtr f1prox_joint = model->GetJoint(urdf_prefix + "finger_1/prox_joint");
	if (f1prox_joint)
	{
		joints_idx_.push_back(0);
		joint_names_.push_back(f1prox_joint->GetName());
		model_joints_.push_back(f1prox_joint);
	}
	else
	{
		RTT::log(RTT::Error) << "Joint [" << (urdf_prefix + "finger_1/prox_joint") << "] not found!" << RTT::endlog();
		return false;
	}

	gazebo::physics::JointPtr f2prox_joint = model->GetJoint(urdf_prefix + "finger_2/prox_joint");
	if (f2prox_joint)
	{
		joints_idx_.push_back(1);
		joint_names_.push_back(f2prox_joint->GetName());
		model_joints_.push_back(f2prox_joint);
	}
	else
	{
		RTT::log(RTT::Error) << "Joint [" << (urdf_prefix + "finger_2/prox_joint") << "] not found!" << RTT::endlog();
		return false;
	}
	gazebo::physics::JointPtr f1med_joint = model->GetJoint(urdf_prefix + "finger_1/med_joint");
	if (f1med_joint)
	{
		joints_idx_.push_back(2);
		joint_names_.push_back(f1med_joint->GetName());
		model_joints_.push_back(f1med_joint);
	}
	else
	{
		RTT::log(RTT::Error) << "Joint [" << (urdf_prefix + "finger_1/med_joint") << "] not found!" << RTT::endlog();
		return false;
	}
	gazebo::physics::JointPtr f2med_joint = model->GetJoint(urdf_prefix + "finger_2/med_joint");
	if (f2med_joint)
	{
		joints_idx_.push_back(3);
		joint_names_.push_back(f2med_joint->GetName());
		model_joints_.push_back(f2med_joint);
	}
	else
	{
		RTT::log(RTT::Error) << "Joint [" << (urdf_prefix + "finger_2/med_joint") << "] not found!" << RTT::endlog();
		return false;
	}
	gazebo::physics::JointPtr f3med_joint = model->GetJoint(urdf_prefix + "finger_3/med_joint");
	if (f3med_joint)
	{
		joints_idx_.push_back(4);
		joint_names_.push_back(f3med_joint->GetName());
		model_joints_.push_back(f3med_joint);
	}
	else
	{
		RTT::log(RTT::Error) << "Joint [" << (urdf_prefix + "finger_3/med_joint") << "] not found!" << RTT::endlog();
		return false;
	}
	gazebo::physics::JointPtr f1dist_joint = model->GetJoint(urdf_prefix + "finger_1/dist_joint");
	if (f1dist_joint)
	{
		joints_idx_.push_back(5);
		joint_names_.push_back(f1dist_joint->GetName());
		model_joints_.push_back(f1dist_joint);
	}
	else
	{
		RTT::log(RTT::Error) << "Joint [" << (urdf_prefix + "finger_1/dist_joint") << "] not found!" << RTT::endlog();
		return false;
	}
	gazebo::physics::JointPtr f2dist_joint = model->GetJoint(urdf_prefix + "finger_2/dist_joint");
	if (f2dist_joint)
	{
		joints_idx_.push_back(6);
		joint_names_.push_back(f2dist_joint->GetName());
		model_joints_.push_back(f2dist_joint);
	}
	else
	{
		RTT::log(RTT::Error) << "Joint [" << (urdf_prefix + "finger_2/dist_joint") << "] not found!" << RTT::endlog();
		return false;
	}
	gazebo::physics::JointPtr f3dist_joint = model->GetJoint(urdf_prefix + "finger_3/dist_joint");
	if (f3dist_joint)
	{
		joints_idx_.push_back(7);
		joint_names_.push_back(f3dist_joint->GetName());
		model_joints_.push_back(f3dist_joint);
	}
	else
	{
		RTT::log(RTT::Error) << "Joint [" << (urdf_prefix + "finger_3/dist_joint") << "] not found!" << RTT::endlog();
		return false;
	}

	link_torque.setZero(model_joints_.size());
	fingertip_torque.setZero(model_joints_.size());
	for (uint i = 0; i < model_joints_.size(); i++)
	{
		torque_switches.push_back(false);
	}

	return true;
}

// bool BarrettHandSim::registerSensors()
// {
// 	frontSensor = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(gazebo::sensors::get_sensor("laser_front"));
// 	backSensor = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(gazebo::sensors::get_sensor("laser_back"));

// 	frontSensor->SetActive(true);
// 	backSensor->SetActive(true);

// 	sensorsRegistered = true;

// 	return sensorsRegistered;
// }

// void BarrettHandSim::getLaserData()
// {
// 	resultVector = Eigen::Vector3d::Zero();
// 	processLaserData(frontSensor, frontSensor->LaserShape());
// 	processLaserData(backSensor, backSensor->LaserShape());
// 	out_force_fdb = rstrt::kinematics::Twist(resultVector(0), resultVector(1), 0, 0, 0, 0);
// 	out_force_fdb_port.write(out_force_fdb);

// 	//From force to acceleration
// 	resultVector = resultVector / 270;
// }

// void BarrettHandSim::processLaserData(gazebo::sensors::RaySensorPtr sensor, gazebo::physics::MultiRayShapePtr rayShape)
// {
// 	for (int rayIndex = 0; rayIndex < sensor->RayCount(); rayIndex++)
// 	{
// 		//tmp3 = angle
// 		tmp(3) = rayShape->GetMinAngle().Radian() + ((rayIndex)*sensor->AngleResolution());
// 		computeForceInfluence(tmp(3));
// 		tmp(0) = rayShape->GetRange(rayIndex);
// 		if (tmp(0) - omnirobRadius < dMax)
// 		{
// 			//Compute position (cos(angle) * dist = x, sin(angle) * dist = y)
// 			tmp(1) = tmp(0) * cos(tmp(3)) + sensor->Pose().Pos().X();
// 			tmp(2) = tmp(0) * sin(tmp(3)) + sensor->Pose().Pos().Y();

// 			out_obstacle_fdb = rstrt::geometry::Translation(tmp(1), tmp(2), 0);
// 			out_obstacle_port.write(out_obstacle_fdb);

// 			resultVector = resultVector + computeForceVector(tmp(0) - omnirobRadius, tmp(3));
// 		}
// 	}
// }

bool BarrettHandSim::setControlMode(std::string controlMode)
{
	// if (controlMode != ControlModes::VelocityCtrl &&
	// controlMode != ControlModes::PositionCtrl)
	// {
	// RTT::log(RTT::Warning) << "Control Mode " << controlMode << " does not exist!" << RTT::endlog();
	// return false;
	// }
	// else
	// {
	RTT::log(RTT::Warning) << "Setting current control mode to " << controlMode << RTT::endlog();
	currentControlMode = controlMode;
	return true;
	// }
}

void BarrettHandSim::initialize()
{
	init_state = INIT_FINGERS;
	// run_mode = INITIALIZE;
	run_mode = RUN; // TODO DLW WAIT FOR CONVERGENCE! Close loop with position feedback here.
}

void BarrettHandSim::idle()
{
	run_mode = IDLE;
	// joint_cmd.mode.assign(oro_barrett_msgs::BHandCmd::MODE_IDLE);
}

void BarrettHandSim::run()
{
	run_mode = RUN;
}

void BarrettHandSim::setCompliance(bool enable)
{
	compliance_enabled = enable;
}

bool BarrettHandSim::withinTorqueLimits(const unsigned joint_id)
{
	return std::abs(joint_torque[joint_id]) > joint_torque_max[joint_id];
}

void BarrettHandSim::readSim()
{
	// Get the state from ALL gazebo joints, not only from the ones that can be actively commanded!
	for (unsigned j = 0; j < model_joints_.size(); j++)
	{
		out_hand_JointFeedback.angles[j] = model_joints_[j]->GetAngle(0).Radian();
		out_hand_JointFeedback.velocities[j] = /* 0.9 * out_hand_JointFeedback.velocities[j] + 0.1 * */ model_joints_[j]->GetVelocity(0);
		out_hand_JointFeedback.torques[j] = model_joints_[j]->GetForce(0u);
	}
	// RTT::log(RTT::Error) << "reading from sim: " << out_hand_JointFeedback << RTT::endlog();
}

void BarrettHandSim::fixAngles()
{
	// Get the finger indices
	unsigned mid, did;
	for (unsigned i = 0; i < 4; i++)
	{
		fingerToJointIDs(i, mid, did);
		if (i == 3)
		{
			in_hand_JointPositionCtrl.angles(3) = out_hand_JointFeedback.angles(1);
		}
		in_hand_JointPositionCtrl.angles(i) = out_hand_JointFeedback.angles(mid);
	}
	b_angle = 0;
	setControlMode(ControlModes::PositionCtrl);
}

void BarrettHandSim::writeSim()
{
	// Transmission ratio between two finger joints
	static const double FINGER_JOINT_RATIO = 1.0 / 3.0;

	for (unsigned i = 0; i < 4; i++)
	{
		// Joint torque
		double joint_torque = 0;

		// Get the finger indices
		unsigned mid, did;
		fingerToJointIDs(i, mid, did);

		double pos = out_hand_JointFeedback.angles[mid] + (i == 3 ? 0 : out_hand_JointFeedback.angles[did]);
		double vel = out_hand_JointFeedback.velocities[mid] + (i == 3 ? 0 : out_hand_JointFeedback.velocities[did]);

		// TODO DLW check if idle
		// case oro_barrett_msgs::BHandCmd::MODE_IDLE:
		// {
		// 	joint_torque = 0.0;
		// 	break;
		// }

		// RTT::log(RTT::Error) << "currentControlMode: " << currentControlMode << RTT::endlog();

		if (currentControlMode == ControlModes::PositionCtrl)
		{
			// joint_torque = p_gain * (in_hand_JointPositionCtrl.angles[i] - pos) - d_gain * vel;
			if (i == 3)
			{
				model_joints_[0]->SetPosition(0, -in_hand_JointPositionCtrl.angles[3]);
				model_joints_[1]->SetPosition(0, in_hand_JointPositionCtrl.angles[3]);
			}
			else
			{
				model_joints_[mid]->SetPosition(0, in_hand_JointPositionCtrl.angles[i]);
				// Keep the joint at the angle it was while it was still tightening
				model_joints_[did]->SetPosition(0, b_angle); //breakaway_angle[i] - out_hand_JointFeedback.angles[did]);
			}
			continue;
			// break;
		}
		// case oro_barrett_msgs::BHandCmd::MODE_TRAPEZOIDAL:
		// {
		// 	const RTT::Seconds sample_secs = (rtt_rosclock::rtt_now() - trap_start_times[i]).toSec();
		// 	joint_torque =
		// 		p_gain * (trap_generators[i].Pos(sample_secs) - pos) +
		// 		d_gain * (trap_generators[i].Vel(sample_secs) - vel);
		// 	break;
		// }
		else if (currentControlMode == ControlModes::VelocityCtrl)
		{
			joint_torque = velocity_gain * (in_hand_JointVelocityCtrl.velocities[i] - vel);
		}
		else if (currentControlMode == ControlModes::TorqueCtrl)
		{
			joint_torque = in_hand_JointTorqueCtrl.torques[i];
		}

		// return; // TODO test

		if (i == 3)
		{
			// Spread: both proximal joints should be kept at the same position
			double spread_err = out_hand_JointFeedback.angles[0] - out_hand_JointFeedback.angles[1];
			double spread_derr = out_hand_JointFeedback.velocities[0] - out_hand_JointFeedback.velocities[1];
			double spread_constraint_force = 100.0 * spread_err + 0.0 * spread_derr;
			model_joints_[0]->SetForce(0, -spread_constraint_force + joint_torque);
			model_joints_[1]->SetForce(0, spread_constraint_force + joint_torque);
		}
		else
		{
			// Fingers: handle TorqueSwitch semi-underactuated behavior

			// Define the gain used to couple the two finger joints
			const double KNUCKLE_GAIN = 10.0;

			// Get link and fingertip torque
			link_torque[i] = model_joints_[mid]->GetForceTorque(0).body2Torque.z;
			fingertip_torque[i] = model_joints_[did]->GetForceTorque(0).body2Torque.z;

			// RTT::log(RTT::Error) << "link_torque " << link_torque << RTT::endlog();
			// RTT::log(RTT::Error) << "fingertip_torque " << fingertip_torque << RTT::endlog();

			// NOTE: The below threshold is too simple and does not capture the
			// real behavior.

			// TorqueSwitch simulation
			//
			// The following code aims to reproduce a behavior similar to the
			// TorqueSwitch mechanism of the real Barrett Hand.
			//
			// Before the inner link's motion is obstructed, the outer link's
			// position is linearly coupled to it by the finger joint ratio.
			//
			// When the inner link encounters a torque larger than the "breakaway"
			// torque, the TorqueSwitch engages and the outer link begins to move
			// inward independently.

			if (!torque_switches[i])
			{
				// RTT::log(RTT::Error) << link_torque[i] << " > " << breakaway_torque << RTT::endlog();
				// Check for torque switch engage condition
				if (link_torque[i] > breakaway_torque)
				{
					// RTT::log(RTT::Error) << "Enabling torque switch for F" << i + 1 << RTT::endlog();
					torque_switches[i] = true;
				}
			}
			else
			{
				// Check for torque switch disengage condition
				if (joint_torque < 0 && out_hand_JointFeedback.angles[mid] > 0.01)
				{
					// RTT::log(RTT::Error) << "Disabling torque switch for F" << i + 1 << RTT::endlog();
					torque_switches[i] = false;
				}
			}

			if (!torque_switches[i])
			{
				// Torque switch has not broken away
				model_joints_[mid]->SetForce(0, joint_torque);
				// Distal joint position is coupled with median joint position
				model_joints_[did]->SetForce(0, KNUCKLE_GAIN * (FINGER_JOINT_RATIO * out_hand_JointFeedback.angles[mid] - out_hand_JointFeedback.angles[did]));
			}
			else
			{
				// Torque switch is in breakaway
				if (joint_torque > 0)
				{
					model_joints_[mid]->SetForce(0, breakaway_torque);
					model_joints_[did]->SetForce(0, FINGER_JOINT_RATIO * joint_torque);
					// Update the position during breakaway
					breakaway_angle[i] = out_hand_JointFeedback.angles[did];
				}
				else
				{
					model_joints_[mid]->SetForce(0, joint_torque);
					// Keep the joint at the angle it was while it was still tightening
					model_joints_[did]->SetForce(0, KNUCKLE_GAIN * (breakaway_angle[i] - out_hand_JointFeedback.angles[did]));
				}
			}
		}
	}
}

void BarrettHandSim::writeFeedbackToOrocos()
{
	// // Always compute and write center of mass
	// this->computeCenterOfMass(center_of_mass);
	// center_of_mass_out.write(center_of_mass);

	this->out_hand_JointFeedback_port.write(out_hand_JointFeedback);

	this->out_hand_FT_port.write(out_hand_FT);

	// // Create a pose structure from the center of mass
	// com_msg.pose.position.x = center_of_mass[0];
	// com_msg.pose.position.y = center_of_mass[1];
	// com_msg.pose.position.z = center_of_mass[2];
	// this->center_of_mass_debug_out.write(com_msg);

	// // Write out hand status
	// this->status_msg.header.stamp = this->joint_state.header.stamp;
	// this->status_msg.temperature.assign(25.0);
	// for (unsigned i = 0; i < 4; i++)
	// {
	// 	this->status_msg.mode[i] = this->joint_cmd.mode[i];
	// }
	// this->status_out.write(this->status_msg);
}

void BarrettHandSim::readCommandsFromOrocos()
{
	switch (run_mode)
	{
	case IDLE:
		// RTT::log(RTT::Error) << "IDLE" << RTT::endlog();
		// Don't command the hand
		break;
	case INITIALIZE:
	{
		switch (init_state)
		{
		case INIT_FINGERS:
			RTT::log(RTT::Error) << "INITIALIZE -> INIT_FINGERS" << RTT::endlog();
			init_state = SEEK_FINGERS;
			break;
		case SEEK_FINGERS:
			RTT::log(RTT::Error) << "INITIALIZE -> SEEK_FINGERS" << RTT::endlog();
			if (doneMoving(0) && doneMoving(1) && doneMoving(2))
			{
				RTT::log(RTT::Error) << "INITIALIZE -> SEEK_FINGERS (done moving)" << RTT::endlog();
				init_state = SEEK_SPREAD;
			}
			break;
		case SEEK_SPREAD:
			RTT::log(RTT::Error) << "INITIALIZE -> SEEK_SPREAD" << RTT::endlog();
			if (doneMoving(3))
			{
				RTT::log(RTT::Error) << "INITIALIZE -> SEEK_SPREAD (done moving)" << RTT::endlog();
				init_state = INIT_CLOSE;
			}
			break;
		case INIT_CLOSE:
			RTT::log(RTT::Error) << "INIT_CLOSE" << RTT::endlog();
			// Increase loop rate
			close();
			run_mode = RUN;
			break;
		};
		break;
	}
	case RUN:
	{
		// Read commands
		bool new_torque_cmd = (in_hand_JointTorqueCtrl_port.readNewest(in_hand_JointTorqueCtrl_new) == RTT::NewData);
		bool new_position_cmd = (in_hand_JointPositionCtrl_port.readNewest(in_hand_JointPositionCtrl_new) == RTT::NewData);
		bool new_velocity_cmd = (in_hand_JointVelocityCtrl_port.readNewest(in_hand_JointVelocityCtrl_new) == RTT::NewData);
		// bool new_trapezoidal_cmd = (joint_trapezoidal_in.readNewest(joint_trapezoidal_cmd) == RTT::NewData);

		// Check sizes
		if (in_hand_JointTorqueCtrl_new.torques.size() != N_PUCKS ||
			in_hand_JointPositionCtrl_new.angles.size() != N_PUCKS ||
			in_hand_JointVelocityCtrl_new.velocities.size() != N_PUCKS)
		// || joint_trapezoidal_cmd.size() != N_PUCKS)
		{
			RTT::log(RTT::Error) << "Input command size mismatch!" << RTT::endlog();
			return;
		}

		// for (int i = 0; i < N_PUCKS; i++)
		// {
		// 	// // Update command vectors with input from ROS message
		// 	// if (new_joint_cmd)
		// 	// {
		// 	// 	switch (joint_cmd_tmp.mode[i])
		// 	// 	{
		// 	// 	case oro_barrett_msgs::BHandCmd::MODE_SAME:
		// 	// 		continue;
		// 	// 	case oro_barrett_msgs::BHandCmd::MODE_IDLE:
		// 	// 		joint_torque_cmd[i] = 0;
		// 	// 		new_torque_cmd = true;
		// 	// 		break;
		// 	// 	case oro_barrett_msgs::BHandCmd::MODE_TORQUE:
		// 	// 		joint_torque_cmd[i] = joint_cmd_tmp.cmd[i];
		// 	// 		new_torque_cmd = true;
		// 	// 		break;
		// 	// 	case oro_barrett_msgs::BHandCmd::MODE_PID:
		// 	// 		joint_position_cmd[i] = joint_cmd_tmp.cmd[i];
		// 	// 		new_position_cmd = true;
		// 	// 		break;
		// 	// 	case oro_barrett_msgs::BHandCmd::MODE_VELOCITY:
		// 	// 		joint_velocity_cmd[i] = joint_cmd_tmp.cmd[i];
		// 	// 		new_velocity_cmd = true;
		// 	// 		break;
		// 	// 	case oro_barrett_msgs::BHandCmd::MODE_TRAPEZOIDAL:
		// 	// 		joint_trapezoidal_cmd[i] = joint_cmd_tmp.cmd[i];
		// 	// 		new_trapezoidal_cmd = true;
		// 	// 		break;
		// 	// 	default:
		// 	// 		RTT::log(RTT::Error) << "Bad BHand command mode: " << (int)joint_cmd_tmp.mode[i] << RTT::endlog();
		// 	// 		return;
		// 	// 	};
		// 	// 	// Set the new command mode
		// 	// 	joint_cmd.mode[i] = joint_cmd_tmp.mode[i];
		// 	// }

		// Update the command
		if (new_torque_cmd && currentControlMode == ControlModes::TorqueCtrl)
		{
			in_hand_JointTorqueCtrl = in_hand_JointTorqueCtrl_new;
		}
		else if (new_position_cmd && currentControlMode == ControlModes::PositionCtrl)
		{
			in_hand_JointPositionCtrl = in_hand_JointPositionCtrl_new;
		}
		else if (new_velocity_cmd && currentControlMode == ControlModes::VelocityCtrl)
		{
			in_hand_JointVelocityCtrl = in_hand_JointVelocityCtrl_new;
		}
		// 	else if (new_trapezoidal_cmd && joint_cmd.mode[i] == oro_barrett_msgs::BHandCmd::MODE_TRAPEZOIDAL)
		// 	{
		// 		joint_cmd.cmd[i] = joint_trapezoidal_cmd[i];
		// 		// Generate trapezoidal profile generators
		// 		unsigned medial_id = 0, distal_id = 0;
		// 		fingerToJointIDs(i, medial_id, distal_id);
		// 		trap_generators[i].SetProfile(out_hand_JointFeedback.angles[medial_id], joint_cmd.cmd[i]);
		// 		trap_start_times[i] = rtt_rosclock::rtt_now();
		// 	}
		// }
	}
	break;
	};
}

// void BarrettHandSim::setTorqueMode(unsigned int joint_index)
// {
// 	joint_cmd.mode[joint_index] = oro_barrett_msgs::BHandCmd::MODE_TORQUE;
// }
// void BarrettHandSim::setPositionMode(unsigned int joint_index)
// {
// 	joint_cmd.mode[joint_index] = oro_barrett_msgs::BHandCmd::MODE_PID;
// }
// void BarrettHandSim::setVelocityMode(unsigned int joint_index)
// {
// 	joint_cmd.mode[joint_index] = oro_barrett_msgs::BHandCmd::MODE_VELOCITY;
// }
// void BarrettHandSim::setTrapezoidalMode(unsigned int joint_index)
// {
// 	joint_cmd.mode[joint_index] = oro_barrett_msgs::BHandCmd::MODE_TRAPEZOIDAL;
// }
// void BarrettHandSim::setIdleMode(unsigned int joint_index)
// {
// 	joint_cmd.mode[joint_index] = oro_barrett_msgs::BHandCmd::MODE_IDLE;
// }

const bool BarrettHandSim::fingerToJointIDs(const uint finger_id, uint &medial_id, uint &distal_id)
{
	if (finger_id < 3)
	{

		medial_id = finger_id + 2;
		distal_id = finger_id + 5;
	}
	else if (finger_id == 3)
	{
		medial_id = 0;
		distal_id = 1;
	}
	else
	{
		return false;
	}

	return true;
}

bool BarrettHandSim::doneMoving(const unsigned pair_index)
{
	unsigned medial_id, distal_id;
	fingerToJointIDs(pair_index, medial_id, distal_id);

	return out_hand_JointFeedback.velocities[medial_id] < 0.01 && out_hand_JointFeedback.velocities[distal_id] < 0.01;
}

void BarrettHandSim::open()
{
	// Open by setting velocity to open
	setControlMode(ControlModes::VelocityCtrl);
	in_hand_JointVelocityCtrl.velocities[0] = -1.0;
	in_hand_JointVelocityCtrl.velocities[1] = -1.0;
	in_hand_JointVelocityCtrl.velocities[2] = -1.0;
}

void BarrettHandSim::close()
{
	// Close by setting negative velocity
	setControlMode(ControlModes::VelocityCtrl);
	in_hand_JointVelocityCtrl.velocities[0] = 1.0;
	in_hand_JointVelocityCtrl.velocities[1] = 1.0;
	in_hand_JointVelocityCtrl.velocities[2] = 1.0;

	// setControlMode(ControlModes::PositionCtrl);
	// in_hand_JointPositionCtrl.angles[0] = f_angle;
	// in_hand_JointPositionCtrl.angles[1] = f_angle;
	// in_hand_JointPositionCtrl.angles[2] = f_angle;
	// in_hand_JointPositionCtrl.angles[3] = s_angle;
}

void BarrettHandSim::openSpread()
{
	// Open by setting velocity to open
	setControlMode(ControlModes::VelocityCtrl);
	in_hand_JointVelocityCtrl.velocities[3] = -1.0;
}

void BarrettHandSim::closeSpread()
{
	// Close by setting negative velocity
	setControlMode(ControlModes::VelocityCtrl);
	in_hand_JointVelocityCtrl.velocities[3] = 1.0;
}

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(cosima::BarrettHandSim)