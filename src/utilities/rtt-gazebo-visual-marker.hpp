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
#include <boost/shared_ptr.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>

namespace cosima
{

class GazeboVisualMarker : public RTT::TaskContext
{
  public:
	GazeboVisualMarker(std::string const &name);
	bool configureHook();
	void updateHook();
	void WorldUpdateBegin();
	void WorldUpdateEnd();

  protected:
	bool getModel(const std::string &model_name);
	void gazeboUpdateHook(gazebo::physics::ModelPtr model);
	bool gazeboConfigureHook(gazebo::physics::ModelPtr model);

	gazebo::physics::ModelPtr model;
	gazebo::event::ConnectionPtr world_begin;
	gazebo::event::ConnectionPtr world_end;

	// RTT::SendHandle<gazebo::physics::ModelPtr(const std::string &, double)> get_model_handle;

	RTT::InputPort<KDL::Frame> in_frame_port;
	RTT::FlowStatus in_frame_flow;
	KDL::Frame in_frame_var;

	void writeSim();

	void readCommandsFromOrocos();

  private:
	bool is_configured;

	std::string urdf_prefix;
};

} // namespace cosima