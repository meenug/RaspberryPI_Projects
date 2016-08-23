# RaspberryPI_Projects

##command_run.cpp##
&nbsp;Generic command runner Subscriber that can start and stop any ROS command on message value "1" or "0".
    
      message type: std_msgs::String
      message value: "1" or "0"
                   
                   1 - to start command
                   0 - to stop command
 
Before running the code, set the parameter cmd with specific ROS command. eg.

     $ rosparam set cmd ["rosbag","record","-a"] or
     $ rosparam set cmd ["rosbag","record","/tf","/cmd_vel"]

It can also be started with a launchfile like this
     
     <launch>
       <node name="camera_record" pkg="nasa_robot" type="command_run">
         <rosparam param="cmd">["rosbag","record","-a"]</rosparam>
       </node>
     </launch>
