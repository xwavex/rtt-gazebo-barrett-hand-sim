#pragma once

#include <string>
#include <rtt/Port.hpp>

namespace cosima
{

struct ControlModes
{
    static constexpr const char *PositionCtrl = "PositionCtrl";
    static constexpr const char *VelocityCtrl = "VelocityCtrl";
    static constexpr const char *TorqueCtrl = "TorqueCtrl";
    static constexpr const char *TrapezoidalCtrl = "TrapezoidalCtrl";
};

struct FeedbackModes
{
    static constexpr const char *velocityFeedback = "JointVelocity";
    static constexpr const char *positionFeedback = "JointPosition";
};

template <class T>
class jointCtrl
{
  public:
    jointCtrl()
    {
    }

    ~jointCtrl()
    {
    }

    RTT::InputPort<T> orocos_port;
    RTT::FlowStatus joint_cmd_fs;
    T joint_cmd;
};

template <class T>
class jointFeedback
{
  public:
    jointFeedback()
    {
    }

    ~jointFeedback()
    {
    }

    T joint_feedback;
    RTT::OutputPort<T> orocos_port;
};
} // namespace cosima
