#include "ros/ros.h"//ros
#include "std_msgs/String.h"
#include "hps_camera/camera.h"//srv
#include "hps_camera/distance.h"
#include "../include/api.h"//api interface
#include <sstream>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

HPS3D_HandleTypeDef handle;
AsyncIObserver_t My_Observer;
ObstacleConfigTypedef ObstacleConf;
ros::Publisher camera_pub;//Global variable, because the observer callback function needs to be used
ros::Publisher pcdata;

//The observer callback function
void *User_Func(HPS3D_HandleTypeDef *handle, AsyncIObserver_t *event)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	sensor_msgs::PointCloud2 output;
	hps_camera::distance msg;
	//uint16_t distance[MAX_PIX_NUM] = {0};

	if(event->AsyncEvent == ISubject_Event_DataRecvd)
	{
		switch(event->RetPacketType)
		{
            case SIMPLE_ROI_PACKET:
                printf("distance average:%d\n",event->MeasureData.simple_roi_data[0].distance_average);
                break;
            case FULL_ROI_PACKET:
                printf("distance average: %d\n",event->MeasureData.full_roi_data[0].distance_average);
                break;
			case FULL_DEPTH_PACKET: /*point cloud data and depth data*/	
				printf("distance average: %d\n",event->MeasureData.full_depth_data->distance_average);
				if(ros::ok())
				{
					cloud.width = event->MeasureData.point_cloud_data->width;
					cloud.height = event->MeasureData.point_cloud_data->height;
					// True if no points are invalid (e.g., have NaN or Inf values in any of their floating point fields).
              		cloud.is_dense = false; 
					cloud.points.resize(cloud.width * cloud.height);
					for (size_t i = 0; i < cloud.points.size (); ++i)
					{
							cloud.points[i].x  = event->MeasureData.point_cloud_data->point_data[i].x/850.0;
							cloud.points[i].y = event->MeasureData.point_cloud_data->point_data[i].y/850.0;
							cloud.points[i].z = event->MeasureData.point_cloud_data->point_data[i].z/850.0;
					}					
					//Convert the cloud to ROS message
					pcl::toROSMsg(cloud, output);
					output.header.frame_id = "laser";
					output.header.stamp = ros::Time::now();
					//camera_pub.publish(output);	
					//msg.distance_average = event->MeasureData.full_depth_data->distance_average;
					//camera_pub.publish(msg);

					pcdata.publish(output); //output


				}
				break;
			case SIMPLE_DEPTH_PACKET:	
				printf("distance average: %d\n",event->MeasureData.simple_depth_data->distance_average);		
				break;
			case OBSTACLE_PACKET:			
				break;
			case NULL_PACKET:
				printf("null packet\n");
				break;
			default:
				printf("system error!\n");
				break;
		}
	}
}
bool manual_connect(RET_StatusTypeDef ret, HPS3D_HandleTypeDef *handle){
    ret = HPS3D_Connect(handle);
    if (ret != RET_OK) {
        printf("Device connect failed！ret = %d\n", ret);
        return false;
    }
    ret = HPS3D_ConfigInit(handle);
    if (RET_OK != ret) {
        printf("Initialization failed! error code is: %d\n", ret);
        return false;
    }
    else {
        printf("Initialization succeed! \n");
        return true;
    }
    return false;
}

uint8_t auto_connect(RET_StatusTypeDef ret, HPS3D_HandleTypeDef *handle){
    uint8_t connect_number = 0;
    connect_number = HPS3D_AutoConnectAndInitConfigDevice(handle);
    
    if(connect_number == 0)
    {
        printf("No devices have been found, so we close the program. \n");
        return 0;   
    }
    else{
        printf("Devices successfully connected = %d\n",connect_number);
    }
    return connect_number;
}

bool add_observer(RET_StatusTypeDef ret, HPS3D_HandleTypeDef *handle, AsyncIObserver_t *My_Observer, uint8_t ID){
    /*An observer subscribes to an event as a data receive event*/
    My_Observer->AsyncEvent = ISubject_Event_DataRecvd;
    My_Observer->NotifyEnable = true; /*enable observer*/
    My_Observer->ObserverID = ID; /*observer id*/

    ret = HPS3D_AddObserver(&User_Func, handle, My_Observer);
    if(RET_OK != ret)
    {
        printf("observer add failed, error code:%d\n", ret);
        return false;
    }
    else{
        printf("observer add successful \n");
        return true;
    }
    return false;
}

void hdr_config(HPS3D_HandleTypeDef *handle){
    /*Get HDR mode*/
    HDRConf hdr_conf, set_conf;
    HPS3D_GetHDRConfig(handle, &hdr_conf);
    /*set SUPER_HDR mode*/
    HPS3D_SetHDRMode(handle, SUPER_HDR); //rec using super_hdr. usecase for each hdr mode is in software manual
    HPS3D_GetHDRConfig(handle, &hdr_conf);
    printf("hdr mode: %d\n", hdr_conf.hdr_mode); //auto 1, super 2, simple 3

    set_conf.hdr_mode = SUPER_HDR;
    set_conf.super_hdr_frame_number = 4;
    set_conf.super_hdr_max_integration = 2000; //us
    HPS3D_SetHDRConfig(handle, set_conf);
    HPS3D_GetHDRConfig(handle, &hdr_conf);
    printf("super_hdr frame number: %d \nsuper_hdr max integration number: %d \n", hdr_conf.super_hdr_frame_number, hdr_conf.super_hdr_max_integration);

}

void filter_config(){
    HPS3D_SetEdgeDetectionEnable (true);
    if (HPS3D_GetEdgeDetectionEnable() == true){
        printf("Edge Detection Enabled");
    }
    else {
        printf("Edge Detection Disabled");
    }
    HPS3D_SetEdgeDetectionValue (1000);
    //printf("Current Edge Detection Value: %d",HPS3D_GetEdgeDetectionValue());

    /*get distance filter parameter*/
    DistanceFilterConfTypeDef distance_filter_conf, set_conf;
    /*set distance filter parameter*/
    //HPS3D_SetDistanceFilterType(&handle, DISTANCE_FILTER_SIMPLE_KALMAN); /*Simple Kalman filter*/
    //HPS3D_GetDistanceFilterConf(&handle, &distance_filter_conf);
    /*set distance filter parameter*/
    set_conf.kalman_K = 0.3;
    set_conf.kalman_threshold = 200;
    set_conf.num_check = 3;
    //HPS3D_SetSimpleKalman(&handle, set_conf);
    //HPS3D_GetDistanceFilterConf(&handle, &distance_filter_conf);

    /*get smooth filter parameter*/
    SmoothFilterConfTypeDef smooth_filter_conf, set_conf1;
    HPS3D_GetSmoothFilterConf(&handle, &smooth_filter_conf);
    printf("Smooth Filter Config type: %d \n",smooth_filter_conf.type);
    /*set smooth filter parameter*/
    set_conf1.type = SMOOTH_FILTER_DISABLE;//SMOOTH_FILTER_AVERAGE
    set_conf1.arg1 = 200;
    HPS3D_SetSmoothFilter(&handle, set_conf1);
    HPS3D_GetSmoothFilterConf(&handle, &smooth_filter_conf); //1 is smooth fliter.


}

bool pointcloud_config(RET_StatusTypeDef ret, HPS3D_HandleTypeDef *handle){
    OpticalParamConfTypeDef optical_param_conf;
    HPS3D_GetOpticalParamConf(handle, &optical_param_conf);
    printf("viewing hori: %d \nviewing verti: %d \n", optical_param_conf.viewing_angle_horiz,optical_param_conf.viewing_angle_vertical);
    ret = HPS3D_SetOpticalEnable(handle, true);
    if (ret != RET_OK) {
        printf("Optical Enable Failed! ret = %d\n", ret);
        return false;
    }
    ret = HPS3D_SetPointCloudEn(true);
    if (ret != RET_OK){
        printf("Point cloud disabled");
        return false;
    }

    if ( HPS3D_GetPointCloudEn() == true) {
        printf("Point cloud enabled \n");
        return true;
    }

    return false;
}

bool packet_config(RET_StatusTypeDef ret, HPS3D_HandleTypeDef *handle){
    MeasurePacketTypeDef x;
    ret = HPS3D_SetMeasurePacketType(DEPTH_DATA_PACKET); // or ROI_DATA_PACKET
    if (ret != RET_OK) {
        printf("Packet config failed！ret = %d\n", ret);
        return false;
    }
    x = HPS3D_GetMeasurePacketType();
    if (x == 0){
        printf("Measure packet type: DEPTH_DATA_PACKET \n");
    }
    else if (x == 1){
        printf("Measure packet type: ROI_DATA_PACKET \n");
    }

    ret = HPS3D_SetPacketType(handle, PACKET_FULL); // or PACKET_SIMPLE = no dept data
    if (ret != RET_OK) {
        printf("Packet type config failed！ret = %d\n", ret);
        return false;
    }

    ret = HPS3D_GetPacketType(handle);
    if (ret != RET_OK){
        printf("Failed to get packet type");
        return false;
    }
    if(handle->OutputPacketType == 0){
        printf("Output Packet Type: PACKET_FULL \n");
    }
    else if (handle->OutputPacketType == 1){
        printf("Output Packet Type: PACKET_SIMPLE \n");
    }
    return true;
}

//check ctrl+c signal
void signal_handler(int signo)
{
    if(HPS3D_RemoveDevice(&handle) != RET_OK)
    {
		printf("HPS3D_RemoveDevice faild\n");
    }
    else
    {	
        printf("HPS3D_RemoveDevice succeed\n");
    }
    exit(0);
}


//printf log callback function
void my_printf(char *str)
{
	std::cout<< str;
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "ros_camera_client");//ros init
	ros::NodeHandle n;//Create a node
	uint32_t a = 0;
	char fileName[10][20];
	uint32_t dev_cnt = 0;
	uint32_t indx = 0;
	bool verifier;
	RET_StatusTypeDef ret = RET_OK;
	//AsyncIObserver_t My_Observer;

    // Prevent null values in computation by setting ceiling point
    //HPS3D_ConfigSpecialMeasurementValue (true,4000);

	std::stringstream sclient_name;

	//Install the signal
	if(signal(SIGINT,signal_handler) == SIG_ERR || signal(SIGTSTP,signal_handler) == SIG_ERR)
	{
		printf("sigint error");
		return 1;
	}
	
	// create service client
	ros::ServiceClient client = n.serviceClient<hps_camera::camera>("client_login"); 
	hps_camera::camera srv;
	sclient_name<<"camera_client";  
	printf("send name = %s\n",sclient_name.str().c_str());
	srv.request.client_node_name = sclient_name.str(); 

	// Create topic
 	//camera_pub = n.advertise<hps_camera::distance>("camera", 1);
	pcdata = n.advertise<sensor_msgs::PointCloud2>("testpic",1);

	if (client.call(srv)) 
	{ 
		while(ros::ok()) 
		{ 
			printf("rev cmd = %s\n",srv.response.control_cmd.c_str()); 
			if( strcmp(srv.response.control_cmd.c_str(), "start" ) == 0 ) 
				{ 
					break; 
				} 

		} 
	}
	
	//Create a topic
	//camera_pub = n.advertise<sensor_msgs::PointCloud2> ("output", 1);
	
	//set debug enable and install printf log callback function
	HPS3D_SetDebugEnable(false);
	HPS3D_SetDebugFunc(&my_printf);

	packet_config(ret, &handle);
	u_int8_t no_of_device = auto_connect(ret,&handle);
    if (no_of_device < 1){
        return 0; //exit program
    }


    /* Link observer to handle */
    verifier = add_observer(ret, &handle, &My_Observer, 0); //ID 0
    if(!verifier){
        return 0;
    }

    /* To select HDR mode */
    hdr_config(&handle);

    /* To select filters */
    filter_config();

    /* Enable point cloud output */
    verifier = pointcloud_config(ret,&handle);
    if (!verifier){
        printf("Failed to enable point cloud");
        return 0;
    }
    
    /* To begin continuous measurement */
    handle.RunMode = RUN_CONTINUOUS;
    ret = HPS3D_SetRunMode(&handle);
    if (ret != RET_OK){
        printf("Failed to set run mode!");
        return 0;
    }
	
	while(1)
	{
		ros::spinOnce();
		sleep(10);		
	}
		
	return 0;
}

