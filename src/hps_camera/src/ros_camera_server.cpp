#include "ros/ros.h"//ros
#include "hps_camera/distance.h"//msg
#include "hps_camera/camera.h"//srv


bool send_cmd(hps_camera::camera::Request &req, hps_camera::camera::Response
&res)
{
std::stringstream scmd;
printf("client_name: %s\n", req.client_node_name.c_str() );
if( strcmp(req.client_node_name.c_str(), "camera_client" ) == 0 )
{
scmd<< "start";
res.control_cmd = scmd.str();
printf("send_cmd: %s\n", res.control_cmd.c_str() );
}
return true;
}

void chatterCallback(const hps_camera::distance& msg)
{
printf("distance_average = %d\n", msg.distance_average);
} 

int main(int argc, char **argv)
{

ros::init(argc, argv, "ros_camera_server");
ros::NodeHandle n;
ROS_INFO("ACCEPT CLIENTS");
ros::ServiceServer service = n.advertiseService("client_login", send_cmd);
ros::Subscriber sub = n.subscribe("camera", 1000, chatterCallback);
ros::spin(); 
return 0;
}
