#include <iostream>

#include "led_coloring.h"
#include "ros/ros.h"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(LedColoring)

void LedColoring::Load(rendering::VisualPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  
    // Store the pointer to the model
    this->model = _parent;
    // Listen to the update event. This event is broadcast pre-render update???
    this->updateConnection = event::Events::ConnectPreRender(boost::bind(&LedColoring::OnUpdate, this));
    

    
    std::string ns;
    ns = "teste";
   
    
    std::cout << "\n loading plugin Led Coloring for ... " << ns;
}



void LedColoring::OnUpdate()
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


