# CPlusPlus_Tutorials

C++ Tutorials for educational purposes

// ----------------------------------------------------------------------------  
//  
// <b>Pre-requisite: Install ROS2</b>  
//  
// ----------------------------------------------------------------------------  
<a>https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Binary.html</a>

NOTE: do not forget to source your ROS environment:  
&nbsp;&nbsp;&nbsp;&nbsp;<i>source /.../install/setup.bash</i>  

// ----------------------------------------------------------------------------  
//  
// <b>timer_tutorial: tutorial on how to work with condition variables</b>  
//  
// ----------------------------------------------------------------------------  
build:  
&nbsp;&nbsp;&nbsp;&nbsp;<i>colcon build</i>  
    
run:  
&nbsp;&nbsp;&nbsp;&nbsp;<i>ros2 run timer_tutorial timer_tutorial</i>  

trigger timer start:  
&nbsp;&nbsp;&nbsp;&nbsp;<i>ros2 topic pub /timer_tutorial/start_timer std_msgs/msg/UInt32 "data: 5000" --once</i>  


// ----------------------------------------------------------------------------  
//  
// <b>engine_ctrl: tutorial for building a state machine</b>  
//  
// ----------------------------------------------------------------------------  
build:  
&nbsp;&nbsp;&nbsp;&nbsp;<i>colcon build</i>  
    
run:  
&nbsp;&nbsp;&nbsp;&nbsp;<i>ros2 run engine_ctrl engine_ctrl</i>  

start engine:  
&nbsp;&nbsp;&nbsp;&nbsp;<i>ros2 topic pub /engine_ctrl_tutorial/start std_msgs/msg/Bool "data: true" --once</i>  
    
stop engine:  
&nbsp;&nbsp;&nbsp;&nbsp;<i>ros2 topic pub /engine_ctrl_tutorial/start std_msgs/msg/Bool "data: false" --once</i>  
  
  
/// ----------------------------------------------------------------------------  
///  
///&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<b><i>ENGINE STATE MACHINE</i></b>  
///  
/// ----------------------------------------------------------------------------  
///  
///&nbsp;See: <i>"engine_ctrl_solution.h"</i>  
///  
///  
///&nbsp;EVENTS (CALLBACKS)  
///&nbsp;=================  
///&nbsp;E1&nbsp;&nbsp;Engine START request  
///&nbsp;E2&nbsp;&nbsp;Engine STOP request  
///&nbsp;E3&nbsp;&nbsp;Crank Timer Timeout after 5 seconds (and RPM value 1000 NOT achieved)  
///&nbsp;E4&nbsp;&nbsp;RPM >= 1000  
///&nbsp;E5&nbsp;&nbsp;RPM < 1000  
///  
///&nbsp;ACTIONS  
///&nbsp;=======  
///&nbsp;A1&nbsp;&nbsp;START cranking the engine => This will start the engine AND start cranking  
///&nbsp;A2&nbsp;&nbsp;Start timer that times out after 5 seconds  
///&nbsp;A3&nbsp;&nbsp;STOP cranking the engine  
///&nbsp;A4&nbsp;&nbsp;STOP engine  
///  
/// ----------------------------------------------------------------------------  

