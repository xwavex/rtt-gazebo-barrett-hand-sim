#include "rtt-gazebo-visual-marker.hpp"
#include <rtt/Operation.hpp>
#include <string>
#include <fstream>
#include <streambuf>
#include <sstream>

using namespace cosima;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

GazeboVisualMarker::GazeboVisualMarker(const std::string &name) : TaskContext(name), is_configured(false), urdf_prefix("")
{
	this->provides("gazebo")->addOperation("WorldUpdateBegin",
										   &GazeboVisualMarker::WorldUpdateBegin, this, RTT::ClientThread);
	this->provides("gazebo")->addOperation("WorldUpdateEnd",
										   &GazeboVisualMarker::WorldUpdateEnd, this, RTT::ClientThread);

	this->addOperation("getModel", &GazeboVisualMarker::getModel, this, ClientThread);

	this->addProperty("urdf_prefix", urdf_prefix);

	world_begin = gazebo::event::Events::ConnectWorldUpdateBegin(
		boost::bind(&GazeboVisualMarker::WorldUpdateBegin, this));
	world_end = gazebo::event::Events::ConnectWorldUpdateEnd(
		boost::bind(&GazeboVisualMarker::WorldUpdateEnd, this));
}

void GazeboVisualMarker::WorldUpdateBegin()
{
	readCommandsFromOrocos();
	writeSim();
}

void GazeboVisualMarker::WorldUpdateEnd()
{
}

bool GazeboVisualMarker::getModel(const std::string &model_name)
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

void GazeboVisualMarker::updateHook()
{
}

bool GazeboVisualMarker::configureHook()
{
	this->is_configured = gazeboConfigureHook(model);

	this->addPort("in_frame_port", in_frame_port).doc("Input port for world frame of the model.");
	in_frame_flow = RTT::NoData;
	in_frame_var = KDL::Frame();

	return is_configured;
}

bool GazeboVisualMarker::gazeboConfigureHook(gazebo::physics::ModelPtr model)
{
	if (model.get() == NULL)
	{
		RTT::log(RTT::Error) << "Model could not be loaded!" << RTT::endlog();
		return false;
	}

	return true;
}

void GazeboVisualMarker::writeSim()
{
	if ((model) && (in_frame_flow == RTT::NewData))
	{
		gazebo::math::Pose pose;
		pose.pos.Set(in_frame_var.p.x(), in_frame_var.p.y(), in_frame_var.p.z());
		double w, x, y, z;
		in_frame_var.M.GetQuaternion(x, y, z, w);
		pose.rot.Set(w, x, y, z);

		model->SetWorldPose(pose);
	}
}

void GazeboVisualMarker::readCommandsFromOrocos()
{
	in_frame_flow = in_frame_port.read(in_frame_var);
}

// ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(cosima::GazeboVisualMarker)