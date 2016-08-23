/*********************************
* Starts and stops recording with rosbag
* Written by: Meenu Goswami
*
* Subscriber to start recording and stop recording
*      message type: std_msgs::String
*      message value: "1" or "0"
*                   1 - to start recording
*                   0 - to stop recording 
* The bag file is created in the directory this program 
* is run from. 
*
* Before running the code, set the parameter cmd
* with specific rosbag record command. eg.
*     rosparam set cmd ["rosbag","record","-a"] or
*     rosparam set cmd ["rosbag","record","/tf","/cmd_vel"] 
*
* It can also be started with a launchfile like this
* <launch>
*   <node name="camera_record" pkg="nasa_robot" type="command_run">
*     <rosparam param="cmd">["rosbag","record","-a"]</rosparam>
*   </node>
* </launch>
***********************************/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h> 
#include <sys/types.h>
#include <signal.h>
#include <string.h>

using namespace std;

pid_t PID ;           // process id of the process forked
bool started = false; // check if recording is started
char **record_cmd;    // command with arguments
char *cmd_name;       // command to run

// callback function
void image_recorderCallback(const std_msgs::String::ConstPtr& msg)
{
  
  if ( strcmp((const char* ) msg->data.c_str(), "1") == 0 && !started)
  { // start the recording process

    started = true;
    ROS_INFO("Starting recording...");
    PID = fork();

    if (PID < 0){
       started = false;
       ROS_INFO("Error: Creating a child process for rosbag");
    }

    else if(PID == 0) {

       // Use setpgid in the child to set its Group PID equal to its own PID.
       // The parent then can kill(-pid,...) to signal the entire group.
       setpgid(0, 0);

       // looks for command from the directories specified by PATH env. var.
       execvp(cmd_name, record_cmd);
    }
  }

  else if ( strcmp((const char* ) msg->data.c_str(), "0") == 0 && started)
  {
     // kill the group process, SIGINT will do a clean exit of rosbag 
     kill(-PID, SIGINT);  
     started = false;
     ROS_INFO("Stopped recording.");
  }
}

int main(int argc, char **argv)
{

  // node name
  ros::init(argc, argv, "image_recorder");

  // node handler
  // create a new ros::NodeHandle with the private namespace as its namespace
  // The topic and parameter is now under the node's name. 
  // That allows to keep them separate if there are several of those things running in the system.
  ros::NodeHandle n("~");
  ros::Subscriber sub = n.subscribe("run", 1, image_recorderCallback);

  // get parameter that is set with rosbag command 
  XmlRpc::XmlRpcValue cmd;
  n.getParam("cmd", cmd);
  ROS_ASSERT(cmd.getType() == XmlRpc::XmlRpcValue::TypeArray);

  int32_t num_param = cmd.size();
  record_cmd = new char * [num_param + 1]; // extra 1 for NULL
  int32_t i;

  for (i = 0; i < num_param; ++i) 
  {
    ROS_ASSERT(cmd[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    if(i==0) {
       cmd_name = new char[cmd[i].size() + 1]; // NULL terminated string
       strcpy(cmd_name, static_cast<std::string>(cmd[i]).c_str());
    }
       
    record_cmd[i] = new char[cmd[i].size() + 1]; // NULL terminated string
    strcpy(record_cmd[i], static_cast<std::string>(cmd[i]).c_str());

  }

  // terminate command with null for execvp
  record_cmd[i] = (char *)0;

  ros::spin();

  // free the memory
  for (i = 0; i < num_param; ++i) 
  {
     delete [] record_cmd[i];
  }

  delete [] cmd_name;

  // array of pointers
  delete [] record_cmd;

  return 0;
} // main()
