#ifndef _GAZEBO_BIG_LED_PLUGIN_HH_
#define _GAZEBO_BIG_LED_PLUGIN_HH_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
class LedColoring : public VisualPlugin
{
public:
    void Load(rendering::VisualPtr _parent, sdf::ElementPtr /*_sdf*/);
    // Called by the world update start event
    void OnUpdate();

private:
    // Pointer to the bigLed
    rendering::VisualPtr model;
    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;
};
}

#endif // model_coloring.h
