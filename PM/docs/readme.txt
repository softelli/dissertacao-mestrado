Terminal 1
$: roscore

Terminal 2
~/catkin_ws/src/PioneerModel$: roslaunch p3dx_gazebo/launch/gazebo.launch					

/***********************************************************************/
Terminal 3
~/catkin_ws$: rosrun p3dx_mover mover.py

ou

rosrun p3dx_mover control.py p1/    // nesse caso, p1/ eh o namespace do robo a ser controlado


//eh possivel controlar o pionner pelas teclas. 
/**********************************************************************/

Terminal 4
//visualizar o funcionamento do laser
~/catkin_ws$: gztopic view /gazebo/default/p1/base_link/head_hokuyo_sensor/scan

Terminal 5
//visualizar a media da frequencia
~/catkin_ws$: rostopic hz /p3dx/laser/scan

(uma vez inserido o argumento "-namespace p1" no spaw_node de gazebo.launch, o comando para cada laser em casa robo deve ser)

~/catkin_ws$: rostopic echo /p1/p3dx/laser/scan

Testar
rostopic pub -r 10 /p1/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

/***** 
Quando o [rosrun] nao encontra o executavel, pode ser necessario utilizar chmod +x no aquivo.py

/****
Resolver o problema do catkin_make CMake Error at ...: 75
https://github.com/cvra/roscvra/issues/4

/****
To start simulation, run

$ cd ~/gazebo_plugin_tutorial/
$ gzserver -u model_push.world

The -u option starts the server in a paused state.

In a separate terminal, start the gui

$ gzclient

Click on the play button in the gui to unpause the simulation, ...



