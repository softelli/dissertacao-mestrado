#include <iostream>

#include "big_led.h"


using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(BigLedPlugin)

BigLedPlugin::BigLedPlugin()
{
  gzwarn << "BigLedPlugin created!\n";
  this->rosNode = NULL;
}

BigLedPlugin::~BigLedPlugin()
{
  gzwarn << "BigLedPlugin deleted..\n";
  delete this->rosNode;
}

void BigLedPlugin::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
{
    gzwarn << "BigLedPlugin in Load()...\n";
    
    // Store the pointer to the model
    this->model = _parent;
    // Listen to the update event. This event is broadcast pre-render update???
    //this->updateConnection = event::Events::ConnectPreRender(boost::bind(&BigLedPlugin::OnUpdate, this));
    
    //cmd_big_led = this->rosNode->subscribe<std_msgs::Int8>("/setLedColor", 1, boost::bind(&BigLedPlugin::cmdSetLedColor, this, _1));
   cmd_big_led = this->rosNode->subscribe<std_msgs::Int8>("/setLedColor", 1, boost::bind(&BigLedPlugin::cmdSetLedColor, this, _1));
      
    if(_sdf->HasElement("center_wheel"))
    {
      gzwarn << "Element big_led founded...\n";
    } 
    else 
    {
      gzwarn << "Element big_led not founded...\n";
    }
    
    std::string ns;
    ns = "teste";
      
    std::cout << "\n loading BigLedPlugin for ... " << ns << std::endl;
}

void BigLedPlugin::cmdSetLedColor(const std_msgs::Int8::ConstPtr& cmd_msg) {
    gzwarn << "Received Led Color Code " << cmd_msg;
}

void BigLedPlugin::OnUpdate()
{
    static float r=0.0, increment=0.01;
    //std::cout << r;
    if (r > 1.0) {
        increment = -0.01;
        r = 1.0;
    }
    if (r < 0.0) {
        r = 0.0;
        increment = 0.01;
    }
    //
    common::Color c(r, 0.0, 0.0);
    this->model->SetAmbient(c);
    this->model->SetDiffuse(c);
    //
    r += increment;
};


