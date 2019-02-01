#include "rtt-gazebo-barrett-hand-sim.hpp"
#include <Eigen/Dense>

using namespace cosima;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

void BarrettHandSim::WorldUpdateBegin()
{
	if (!is_configured && !isRunning())
		return;

	// baseOrientation = model->GetWorldPose().rot;
	// basePose = model->GetWorldPose().pos;
	// if (sensorsRegistered)
	// {
	// 	getLaserData();
	// }
	// control(Eigen::Vector3d(basePose.x, basePose.y, baseOrientation.GetYaw()), Eigen::Vector3d(model->GetWorldLinearVel().x, model->GetWorldLinearVel().y, model->GetWorldAngularVel().z));

	// TODO check order
	readSim();
	readCommandsFromOrocos();
}

void BarrettHandSim::WorldUpdateEnd()
{
	if (!is_configured && !isRunning())
		return;

	// model->SetLinearVel(newLinVel);
	// model->SetAngularVel(newAngVel);
	// Read from FT sensor
	if (parentJointForFT)
	{
		gazebo::physics::JointWrench wrench = this->parentJointForFT->GetForceTorque(0u);

		gazebo::math::Vector3 measuredForce = wrench.body2Force;
		gazebo::math::Vector3 measuredTorque = wrench.body2Torque;

		// RTT::log(RTT::Error) << "reading from sim:\n2:\n"
		// 					 << wrench.body2Force << "\n1:\n"
		// 					 << (-1 * wrench.body1Force) << RTT::endlog();

		if (zeroing_active)
		{
			zero_hand_FT.forces(0) = measuredForce.x;
			zero_hand_FT.forces(1) = measuredForce.y;
			zero_hand_FT.forces(2) = measuredForce.z;

			zero_hand_FT.torques(0) = measuredTorque.x;
			zero_hand_FT.torques(1) = measuredTorque.y;
			zero_hand_FT.torques(2) = measuredTorque.z;
			zeroing_active = false;
		}

		out_hand_FT.forces(0) = measuredForce.x - zero_hand_FT.forces(0);
		out_hand_FT.forces(1) = measuredForce.y - zero_hand_FT.forces(1);
		out_hand_FT.forces(2) = measuredForce.z - zero_hand_FT.forces(2);

		out_hand_FT.torques(0) = measuredTorque.x - zero_hand_FT.torques(0);
		out_hand_FT.torques(1) = measuredTorque.y - zero_hand_FT.torques(1);
		out_hand_FT.torques(2) = measuredTorque.z - zero_hand_FT.torques(2);
	}
	// TODO check order
	writeSim();
	writeFeedbackToOrocos();

	// in_velocity_cmd_flow = in_velocity_cmd_port.readNewest(in_velocity_cmd);
	// if (in_velocity_cmd_flow == RTT::NewData)
	// {
	// 	desiredTwist = Eigen::Vector3d(in_velocity_cmd.linear(0), in_velocity_cmd.linear(1), in_velocity_cmd.angular(3));
	// }

	// in_relativePose_cmd_flow = in_relativePose_cmd_port.readNewest(in_relativePose_cmd);
	// if (in_relativePose_cmd_flow == RTT::NewData)
	// {
	// 	desiredPose = Eigen::Vector3d(basePose.x + in_relativePose_cmd.translation.translation(0), basePose.y + in_relativePose_cmd.translation.translation(1), baseOrientation.GetYaw() + in_relativePose_cmd.rotation.rotation(4));
	// }

	// in_pose_cmd_flow = in_pose_cmd_port.readNewest(in_pose_cmd);
	// if (in_pose_cmd_flow == RTT::NewData)
	// {
	// 	desiredPose = Eigen::Vector3d(in_pose_cmd.translation.translation(0), in_pose_cmd.translation.translation(1), in_pose_cmd.rotation.rotation(4));
	// }
}
