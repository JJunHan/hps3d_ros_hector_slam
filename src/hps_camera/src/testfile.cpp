/**
 * @file testfile.cpp
 * @author Radar Team
 * @brief Welcome :)
 * @version 0.1
 * @date 2021-06-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include "api.h"
#include <unistd.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <dirent.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdarg.h>
#include <math.h>
#include <set>
#include <pcl/ml/kmeans.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <iostream>

/*
 * Main declaration of important variables
 * Wrap in std::vector --> dynamic array, possible to include more lidar next time. 
 */
HPS3D_HandleTypeDef handle;
AsyncIObserver_t My_Observer;
std::vector < pcl::PointCloud<pcl::PointXYZ>::Ptr > pointCloudPtr;
std::vector < pcl::visualization::PCLVisualizer::Ptr > viewersPtr;

/*
* User processing function,Continuous measurement or asynchronous mode
* in which the observer notifies the callback function
* */
void* event_handler(HPS3D_HandleTypeDef *handle,AsyncIObserver_t *event)
{
    
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
            case FULL_DEPTH_PACKET:
                //printf("distance average: %d\n",event->MeasureData.full_depth_data->distance_average);
                
                // Find specifications in point_cloud.h
                pointCloudPtr[event->ObserverID]->width = event->MeasureData.point_cloud_data->width;
                pointCloudPtr[event->ObserverID]->height = event->MeasureData.point_cloud_data->height;
                // True if no points are invalid (e.g., have NaN or Inf values in any of their floating point fields).
                pointCloudPtr[event->ObserverID]->is_dense = false; 
                pointCloudPtr[event->ObserverID]->points.resize(pointCloudPtr[event->ObserverID]->width * pointCloudPtr[event->ObserverID]->height);
                // Fill the PCL cloud data 
                for (size_t i = 0; i < pointCloudPtr[event->ObserverID]->points.size(); ++i){
                    pointCloudPtr[event->ObserverID]->points[i].x = event->MeasureData.point_cloud_data->point_data[i].x;
                    pointCloudPtr[event->ObserverID]->points[i].y = event->MeasureData.point_cloud_data->point_data[i].y;
                    pointCloudPtr[event->ObserverID]->points[i].z = event->MeasureData.point_cloud_data->point_data[i].z;
                }
                break;
            case SIMPLE_DEPTH_PACKET:
                printf("distance average: %d\n",event->MeasureData.simple_depth_data->distance_average);
                break;
            case NULL_PACKET:
                break;
            default:
                printf("system error\n");
                break;
        }
    }
    return 0;
}

void* singlemeasurement(RET_StatusTypeDef ret, HPS3D_HandleTypeDef *handle){
    ret = RET_OK;
    ret = HPS3D_SingleMeasurement(handle);

    if(ret == RET_OK)
    {
        switch(handle->RetPacketType)
        {
            case SIMPLE_ROI_PACKET:
                printf("Simple Roi measure distance average:%d\n",handle->MeasureData.simple_roi_data[0].distance_average);
                break;
            case FULL_ROI_PACKET:
                printf("Full Roi measure distance average:%d \n",handle->MeasureData.full_roi_data[0].distance_average);
                break;
            case FULL_DEPTH_PACKET:
                printf("Full depth measure distance average:%d \n",handle->MeasureData.full_depth_data->distance_average);
                //HPS3D_SavePlyFile("pointCloud.ply", *(handle->MeasureData.point_cloud_data));
                //processing to be done here
                // Find specifications in point_cloud.h
                pointCloudPtr[0]->width = handle->MeasureData.point_cloud_data->width;
                pointCloudPtr[0]->height = handle->MeasureData.point_cloud_data->height;
                // True if no points are invalid (e.g., have NaN or Inf values in any of their floating point fields).
                pointCloudPtr[0]->is_dense = false; 
                pointCloudPtr[0]->points.resize(pointCloudPtr[0]->width * pointCloudPtr[0]->height);
                // Fill the PCL cloud data 
                for (size_t i = 0; i < pointCloudPtr[0]->points.size(); ++i){
                    pointCloudPtr[0]->points[i].x = handle->MeasureData.point_cloud_data->point_data[i].x;
                    pointCloudPtr[0]->points[i].y = handle->MeasureData.point_cloud_data->point_data[i].y;
                    pointCloudPtr[0]->points[i].z = handle->MeasureData.point_cloud_data->point_data[i].z;
                }
                break;
            case SIMPLE_DEPTH_PACKET:
                printf("simple depth measure distance average:%d \n",handle->MeasureData.simple_depth_data->distance_average);
                break;
            case NULL_PACKET:
                printf("return packet is null\n");
                break;
            default:
                printf("system error\n");
                break;
        }
    }
    return 0;
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

    ret = HPS3D_AddObserver(&event_handler, handle, My_Observer);
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

bool save_profile(RET_StatusTypeDef ret, HPS3D_HandleTypeDef *handle){
    ret = HPS3D_ProfileSaveToCurrent(handle); //Save to user configuration and this configuration will be set as the default configuration after restart. 
    if(RET_OK != ret)
    {
        printf("Profile save failed, error code:%d\n", ret);
        return false;
    }
    else{
        printf("Profile save successful \n");
        return true;
    }
    return false;

    /* // Use when needed
    HPS3D_ProfileClearCurrent(&handle); // Clear user configuration, reset to default configuration. 
    HPS3D_ProfileRestoreFactory(&handle); // Reset factory setting
    */
}

void roi_config(HPS3D_HandleTypeDef *handle){ //for further dev 
    uint8_t group_id = 0;
    /*select group ID*/
    HPS3D_SelectROIGroup(handle, 1);
    /*get group ID*/
    HPS3D_GetROIGroupID(handle, &group_id);

    ROIConfTypeDef roi_conf1, roi_conf2;
    HysteresisSingleConfTypeDef hysteresis_conf1, hysteresis_conf2;

    HPS3D_SetROIAlarmType(handle, 1, 1, ROI_ALARM_DISABLE); //roi id 1, threshold id 1
    HPS3D_SetROIReferenceType(handle, 1, 1, ROI_REF_VAILD_AMPLITUDE);

    roi_conf1.roi_id = 0;
    roi_conf1.left_top_x = 10;
    roi_conf1.left_top_y = 10;
    roi_conf1.right_bottom_x = 30;
    roi_conf1.right_bottom_y = 20;
    HPS3D_SetROIRegion(handle, roi_conf1);

    HPS3D_SetROIEnable(handle, 1, true); //roi id 1
    HPS3D_SetROIThresholdEnable(handle, 1, 1, false); //turn this to true if want to use roi threshold
    /*set ROI threshold config*/
    hysteresis_conf1.threshold_value = 20;
    hysteresis_conf1.hysteresis = 100;
    hysteresis_conf1.positive = true;
    HPS3D_SetROIThresholdConf(handle, 1, 1, 60, hysteresis_conf1);

    HPS3D_GetROIConfById(handle, 1, &roi_conf1);
    printf("%d \n",roi_conf1.pixel_number_threshold[1]);
    printf("%d \n",roi_conf1.hysteresis_conf[1].enable);
    printf("%d \n",roi_conf1.hysteresis_conf[1].threshold_value);
    printf("%d \n",roi_conf1.hysteresis_conf[1].positive);
    printf("%d \n",roi_conf1.hysteresis_conf[1].hysteresis);

    uint8_t roi_number, threshold_number;
    HPS3D_GetNumberOfROI(handle, &roi_number, &threshold_number);

    printf("max supported roi number: %d \n max supported threshold number: %d \n",roi_number,threshold_number);

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

void signalHandler(int signo){
    HPS3D_RemoveDevice(&handle);
    printf("Removing device before closing \n");
    exit(0); // safe exit
}

void linedef(pcl::PointXYZ *p1, pcl::PointXYZ *p2, pcl::PointXYZ *p3, pcl::PointXYZ *p4){
    p1->x = -2200;
    p1->y = 0;
    p1->z = -200;
    p2->x = 2200;
    p2->y = 0;
    p2->z = -200;
    p3->x = 2200;
    p3->y = 0;
    p3->z = 3700;
    p4->x = -2200;
    p4->y = 0;
    p4->z = 3700;
}

void ptrdef(u_int8_t no_of_device){
    //initialise pointclouds
    for (int i=0;i<no_of_device;i++) {
            pointCloudPtr[i]=pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        }

    // if only want to initialise 1 lidar and 1 viewer use this, no need to do std::vector
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::visualization::PCLVisualizer::Ptr viewersPtr (new pcl::visualization::PCLVisualizer ("Viewer"));
    
    // initialise viewers
    // find pcl_visualizer settings in pcl_visualizer.h
    for (int i=0;i<no_of_device;i++) {
        viewersPtr[i]=pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Viewer HPS3D-160"));
        std::string name = "Viewer HPS3D-160";
        viewersPtr[i]->setWindowName(name);
        viewersPtr[i]->setBackgroundColor (0, 0, 0);   // RGB 000 is black
        viewersPtr[i]->initCameraParameters();
        //viewersPtr[i]->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
        //viewersPtr[i]->addCoordinateSystem (5.0);
        viewersPtr[i]->setCameraPosition(0, 2000, 9500, 0, 0, 0, 0, -1, 0, 0); //zoom out and flip 180 degrees
    }
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

int main(){
    char devname[] = "/dev/ttyACM0";
    handle.DeviceName = devname;
    RET_StatusTypeDef ret = RET_OK;
    bool verifier;

    /* Ctrl C handler */
    if(signal(SIGINT, signalHandler) == SIG_ERR) {
        printf("sigint error");
        return 1;
    }

    /* Initialise Packet configurations */
    packet_config(ret, &handle);
    
    /* Automatically connect to x number of devices */
    u_int8_t no_of_device = auto_connect(ret,&handle);
    if (no_of_device < 1){
        return 0; //exit program
    }
    
    /* Initialise number of pointclouds & visulizers */
    pointCloudPtr.resize(no_of_device);
    viewersPtr.resize(no_of_device);
    ptrdef(no_of_device);

    /* Link observer to handle */
    verifier = add_observer(ret, &handle, &My_Observer, 0); //ID 0
    if(!verifier){
        return 0;
    }

    /* To setup ROI */
    //roi_config(&handle);

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

    /* To get single measurement */
    //handle.RunMode = RUN_SINGLE_SHOT;
    //HPS3D_SetRunMode(&handle);
    //singlemeasurement(ret,&handle); //used to call a single measurement
    
    /* To save settings into lidar flash */
    //save_profile(ret, &handle);
    
    /* Widely used variables */
    typedef pcl::PointXYZ PointType;
    constexpr unsigned int THRESHOLD = 50; //25-50 at 125ms is good
    constexpr unsigned int UPPERLIMIT = 320;
    constexpr unsigned int window_size = 7;
    constexpr float CAMERAWIDTH = 76.0f;
    constexpr float CAMERAHEIGHT = 32.0f;

    /* Setup the boundary lines */
    static PointType p1,p2,p3,p4;
    linedef(&p1,&p2,&p3,&p4);

    // -----------------------------------------------
    // ------Parameters for Range Image creation------
    // -----------------------------------------------
    float support_size = 0.8f;
    float angularResolution = (float) (  1.0f * (M_PI/180.0f));
    float maxAngleWidth     = (float) (CAMERAWIDTH * (M_PI/180.0f));
    float maxAngleHeight    = (float) (CAMERAHEIGHT * (M_PI/180.0f));
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f); //row pitch yaw (doesnt seem to affect)
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME; // Tells the system that x is facing right, y downwards and the z axis is forward
    
    pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
    pcl::RangeImage& range_image = *range_image_ptr;   
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 100, 100, 250);

    // -----------------------------------------------------
    // ------Parameters for NARF Keypoints Extractions------
    // -----------------------------------------------------
    pcl::RangeImageBorderExtractor range_image_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor);
    pcl::PointCloud<int> keypoint_indices;
    pcl::PointCloud<PointType>::Ptr keypoints_ptr (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>& keypoints = *keypoints_ptr;
    pcl::visualization::PointCloudColorHandlerCustom<PointType> keypoints_color_handler (keypoints_ptr, 255, 0, 0);

    narf_keypoint_detector.getParameters().support_size = support_size;
    narf_keypoint_detector.getParameters().add_points_on_straight_edges = true; //true
    narf_keypoint_detector.getParameters().distance_for_additional_points = 0.5f; //essentially 0.5*0.8 into distance
    narf_keypoint_detector.getParameters().min_distance_between_interest_points = 0.3f;
    narf_keypoint_detector.getParameters().optimal_distance_to_high_surface_change = 0.3f;
    narf_keypoint_detector.getParameters().min_interest_value = 0.2f;
    narf_keypoint_detector.getParameters().min_surface_change_score = 0.2f;
    narf_keypoint_detector.getParameters().optimal_range_image_patch_size = 20;

    // --------------------------------------------
    // ------Parameters for Border Extraction------
    // --------------------------------------------
    pcl::RangeImageBorderExtractor::Ptr border_extractorPtr (new pcl::RangeImageBorderExtractor);
    pcl::RangeImageBorderExtractor& border_extractor = *border_extractorPtr;
    pcl::PointCloud<pcl::BorderDescription>::Ptr border_descriptionsPtr (new pcl::PointCloud<pcl::BorderDescription>);
    pcl::PointCloud<pcl::BorderDescription>& border_descriptions = *border_descriptionsPtr;
    pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
    pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_points_color_handler (border_points_ptr, 0, 255, 0);

    // --------------------------------
    // ------Euclidean Clustering------
    // --------------------------------
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance (200); // trial and error. 200 seems fine
    ec.setMinClusterSize (30);
    ec.setMaxClusterSize (200);

    // ------------------------------
    // ------K Means Clustering------
    // ------------------------------

    //pcl::Kmeans kObj ((pointCloudPtr[0]->points.size()), 3); //default the first PC of 9600, XYZ 3 dim
    //kObj.setClusterSize(10);
    
    // -------------------------------
    // ------Centriod and Vector------
    // -------------------------------
    pcl::PointCloud<PointType>::Ptr cloud_combined (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>& cc = *cloud_combined;
    std::vector < pcl::PointCloud<PointType>::Ptr > cloud_cluster;
    std::vector < PointType > centriod;
    std::vector < std::vector < PointType >  > sliding_centriod(window_size);
    std::vector < std::vector < int> > sliding_sum(window_size);
    std::set <int> remove_me;
    std::vector <int> sum;
    std::vector <int> score;
    std::vector <int> fall_score;
    
    // Prevent null values in computation by setting ceiling point at 8000
    HPS3D_ConfigSpecialMeasurementValue (true,8000);

    // Loop variables
    static unsigned count = 0;
    static unsigned sw_count = 0;
    std::stringstream name;
    std::stringstream holder;
    //const std::string label_motion = "Moving";

    while(!viewersPtr[0]->wasStopped ()){ // might need to add one more checking variable to break the loop
        for (int i=0;i<viewersPtr.size();i++) {
            if (pointCloudPtr[i]->points.size() == 0) {
                printf("[Point Cloud %d] Size cloud = %lu. Not visualising\n", i,pointCloudPtr[i]->points.size() );
            }
            else {
                // Remove old frames
                viewersPtr[i]->removeAllShapes();
                viewersPtr[i]->removeAllPointClouds(); 

                // Count number of loops since launch
                std::stringstream loop;
                loop << "Once per viewer loop: " << count++;

                //viewersPtr[i]->removeShape("text", 0);
                viewersPtr[i]->addText(loop.str(), 20, 50, "text", 0);
                
                // Draw boundary of lidar
                viewersPtr[i]->addLine(p2,p3,0,255,0,"line2",0);
                viewersPtr[i]->addLine(p3,p4,0,0,255,"line3",0);
                viewersPtr[i]->addLine(p4,p1,255,255,255,"line4",0);
                viewersPtr[i]->addLine(p1,p2,255,0,0,"line1",0); // work in progress
                
                
                // Convert PointCLoud to Range Image for processing
                pcl::PointCloud<PointType>& point_cloud = *pointCloudPtr[i];
                range_image.createFromPointCloud (point_cloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                   sensorPose, coordinate_frame, noise_level, min_range, border_size);

                // Visualize the raw Range Image 
                viewersPtr[i]->addPointCloud (range_image_ptr, range_image_color_handler, "range image");
                viewersPtr[i]->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
                
                // Border extraction (another keypoint algo)
                pcl::RangeImageBorderExtractor border_extractor(&range_image);
                border_extractor.compute (border_descriptions);
                //printf("Range image size %lu Pointcloud size %lu\n",range_image.size(),point_cloud.size());
                for (int y=0; y< (int)range_image.height; ++y)
                {
                    for (int x=0; x< (int)range_image.width; ++x)
                    {
                    if (border_descriptions[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
                        border_points.points.push_back (range_image[y*range_image.width + x]);
                    }
                }
                //printf("Number of Border Points: %lu ",border_points.size());
                        
                // Visualize the Border Points
                //viewersPtr[i]->addPointCloud<pcl::PointWithRange> (border_points_ptr, border_points_color_handler, "border points");
                //viewersPtr[i]->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");

                // Compute keypoints using NARF algo
                narf_keypoint_detector.setRangeImage(&range_image);
                narf_keypoint_detector.compute(keypoint_indices);
                //printf("Number of Keypoints %lu \n",keypoint_indices.size());

                keypoints.resize (keypoint_indices.size ());
                for (std::size_t i=0; i<keypoint_indices.size (); ++i)
                    keypoints[i].getVector3fMap () = range_image[keypoint_indices[i]].getVector3fMap ();

                // Visualize the Keypoints
                //viewersPtr[i]->addPointCloud<pcl::PointXYZ> (keypoints_ptr, keypoints_color_handler, "keypoints");
                //viewersPtr[i]->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
                
                // Visualize raw unprocessed PointCloud from lidar
                //name << "HPS3D Lidar " << i+1;
                //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb (pointCloudPtr[i], 0, 0, 255);
                //viewersPtr[i]->addPointCloud<pcl::PointXYZ> (pointCloudPtr[i], rgb, name.str());
                //viewersPtr[i]->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name.str());

                // Clustering data preparations
                // Convert RangeImage into PointType
                cc.width = border_points.size();
                cc.height = 1;
                cc.points.resize(cc.width * cc.height);
                
                for(int m=0;m<border_points.size();++m){
                    cc.points[m].x = border_points.points[m].x;
                    cc.points[m].y = border_points.points[m].y;
                    cc.points[m].z = border_points.points[m].z;
                }

                // Concat Border PC and Keypoint PC
                cc += *(keypoints_ptr);
/*
                // Do kmeans clustering here
                // Fill the cloud
                pcl::Kmeans kObj (cc.points.size(), 3); //default the first PC of 9600, XYZ 3 dim
                kObj.setClusterSize(7);
                for (std::size_t k = 0; k < cc.points.size(); ++k) {
                        std::vector<float> data(3);
                        data[0] = cc[k].x;
                        data[1] = cc[k].y;
                        data[2] = cc[k].z;
                        kObj.addDataPoint(data);
                    }

                //pcl::Kmeans::ClustersToPoints clusters_to_points_;
                kObj.kMeans();
                kObj.computeCentroids();
               
                pcl::Kmeans::Centroids centroids = kObj.get_centroids();
                u_int8_t no_of_real_centroids = centroids.size();
                //printf("size of centriods : %ld \n ",centroids.size());
                cloud_cluster.resize(centroids.size()); // Setup pc vector size to no of clusters detected
                centriod.resize(centroids.size()); // Setup centriod vector size to no of clusters detected
                for(std::size_t g = 0; g<centroids.size(); ++g){
                    if (isinf(centroids[g][0])){
                        centriod[g].x = 0;
                        centriod[g].y = 0;
                        centriod[g].z = 0;
                        continue;
                    }

                    centriod[g].x = centroids[g][0];
                    centriod[g].y = centroids[g][1];
                    centriod[g].z = centroids[g][2];
                    //printf("centriod point xyz %f", centriod[g].x);
                    //printf("X: %lf, Y: %lf, Z: %lf \n", centroids[g][0], centroids[g][1], centroids[g][2]);
                }

                for (int o=0;o<centroids.size();++o) {
                    cloud_cluster[o]=pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
                }        

                //printf("Total points detected: %lu \n",cc.size());
                
                for(int f = 0; f < clusters_to_points_.size(); ++f){
                    
                    //for(int d = 0; d<clusters_to_points_[f].size(); ++d){
                        printf("Size of each cluster = %ld \n", clusters_to_points_[f].size());
                        //clusters_to_points_[f].
                        std::set<pcl::Kmeans::PointId>::iterator setIt = clusters_to_points_[f].begin();
                        printf("Info of each cluster = %u \n",*setIt);
                    //}
                }
                */
                
            

                // Perform Clustering and extract indices which indicates which area of the PC the cluster is at
                tree->setInputCloud (cloud_combined);
                ec.setSearchMethod (tree);
                ec.setInputCloud (cloud_combined);
                ec.extract (cluster_indices);
                
                // Idea here is for each cluster detected, create a point cloud object for us to create vectors
                // push back moves the xyz points of that row into a pc object
                //printf("Total points detected: %lu Total clusters formed: %lu \n",cc.size(),cluster_indices.size());
                cloud_cluster.resize(cluster_indices.size()); // Setup pc vector size to no of clusters detected
                centriod.resize(cluster_indices.size()); // Setup centriod vector size to no of clusters detected
                // Create object for each cluster
                for (int o=0;o<cluster_indices.size();++o) {
                    cloud_cluster[o]=pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
                }        

                // Loop control mechanism
                int j = 0;
                
                for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
                        for (const auto& idx : it->indices) // const auto for read only access to the indices content
                            cloud_cluster[j]->push_back((*cloud_combined)[idx]); // populate the cloud cluster
                        cloud_cluster[j]->width = cloud_cluster[j]->size ();
                        cloud_cluster[j]->height = 1; // or cloud_cluster[j]->size();
                        //printf("PointCloud representing the Cluster: %d has %lu points",j,cloud_cluster[j]->size()); 

                        // Computing Centriods for each cluster and storing into vector
                        pcl::computeCentroid(*cloud_cluster[j],centriod[j]);
                        //printf("x y z %f %f %f \n",centriod[j].x,centriod[j].y,centriod[j].z);
                        
                        // Visualize each cluster as it is found. (All same colour)
                        //holder << "HPS3D Lidars " << j+1; //new name for each cluster
                        //viewersPtr[i]->addPointCloud<pcl::PointXYZ> (cloud_cluster[j], holder.str());
                        //viewersPtr[i]->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, holder.str());
                        j++;
                }

                // Sum all the current frame's cluster's centriods
                for(int k = 0;k<centriod.size(); ++k){
                    sum.push_back(centriod[k].x + centriod[k].y + centriod[k].z); 
                    // uncomment for kmeans cluster
                    //cloud_cluster[k]->push_back(centriod[k]); //1 cloud only one point atm, the centriod.
                    //cloud_cluster[k]->width = cloud_cluster[k]->size();
                    //cloud_cluster[k]->height = 1;
                }

                // Sum all the current cluster centriods
                // Check each element in the current centriod array with that of the previous 2D array
                // Remove all static and new clusters
                
                for(int x = 0; x<sum.size(); ++x){
                    u_int8_t sum_Score = 0;
                    bool newcluster = true;
                    //printf("Original centriod y: %f\n", centriod[x].y);
                    for(int y = 0; y<sliding_sum.size(); ++y){
                        for(int z = 0; z<sliding_sum[y].size(); ++z){
                            if(abs(sum[x] - sliding_sum[y][z]) < THRESHOLD){ // < threshold probly means its not moving
                                remove_me.insert(x); // add the index into the set
                                newcluster = false;
                                break;
                            }
                            else if(abs(sum[x] - sliding_sum[y][z]) < UPPERLIMIT - y*10){ // If an object has deviated away from its original position by an upper limit
                                // Means the object is found in the sliding window and it is moving.
                                //printf("sum score = %u\n", sum_Score);
                                if(y==6){ // If it is the 7th frame must have the most weightage
                                    sum_Score += 24;
                                    
                                } 
                                else {
                                    sum_Score += y*2+1;
                                }
                                newcluster = false;              
                                break; 
                            }

                        } 
                    }
                    if(newcluster){
                        remove_me.insert(x);
                    }
                    // Every Frame's Clusters will generate a score with relation to the current frame's clusters. 
                    score.push_back(sum_Score);                    
                }
                

                // If the index we want to plot matches the one in the remove list then skip.
                // Choose which clusters to plot!
                // Add bounding box
                float xmin, xmax, ymin, ymax, zmin, zmax;
                std::string label_motion = "Moving";
                for(int j = 0; j < centriod.size(); ++j){
                    
                    if(remove_me.count(j)){  // This are confirmed static points
                        continue;
                    }
                    
                    printf("Sum score: %d \n", score[j]);
                    if(score[j] < 24){ // Not enough score to consider it as a moving point.
                        continue;
                    }

                    u_int8_t fall_Score = 0;
                    for(int y = 0; y<sliding_sum.size(); ++y){
                        u_int8_t invert = sliding_sum[y].size();
                        for(int z = 0; z<sliding_sum[y].size(); ++z){
                            if(centriod[j].y > sliding_centriod[y][z].y){ //if centriod y is lower to the ground than the frame's
                                printf("falling centriod y: %f, resultant product: %f\n", sliding_centriod[y][z].y, abs(centriod[j].y - sliding_centriod[y][z].y));
                                if(abs(centriod[j].y - sliding_centriod[y][z].y)> 300-y*10){ //this parameter must change depending on fall distance.
                                    fall_Score += invert*2+1;
                                    invert--; 
                                    //printf("fall score: %d \n", fall_Score);
                                }
                            }
                        }
                    }
                    fall_score.push_back(fall_Score);
                    
                    printf("Fall score: %d \n", fall_score[j]);
                    if(fall_score[j] > 17){ // Consider it as this object is falling or fell.
                        label_motion = "Falling";
                        sw_count++;
                        printf("Fall detected!! Repetition: %d\n", sw_count);
                    }
                    
                    
                    holder << "HPS3D Lidar " << j+1; //new name for each cluster
                    name << "Label " << i+1; //new name for each label
                    viewersPtr[i]->addPointCloud<pcl::PointXYZ> (cloud_cluster[j], holder.str());
                    viewersPtr[i]->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, holder.str());
                    xmin = centriod[j].x - 200.0f;
                    xmax = centriod[j].x + 200.0f;
                    ymin = centriod[j].y - 500.0f;
                    ymax = centriod[j].y + 500.0f;
                    zmin = centriod[j].z - 100.0f;
                    zmax = centriod[j].z + 100.0f;
                    viewersPtr[i]->addCube(xmin,xmax,ymin,ymax,zmin,zmax,1,0,0,holder.str());
                    PointType label_pos = {xmax, ymin, zmax};
                    viewersPtr[i]->addText3D(label_motion,label_pos,100.0,0.0,1.0,0.0,name.str(),0);
                    viewersPtr[i]->setRepresentationToWireframeForAllActors();
                }


                // Remove old frame if sliding window is full
                if(sliding_sum.size() == window_size) {// sliding wind size has reached it's maximum size
                    sliding_sum.erase(sliding_sum.begin()); // remove first element
                    sliding_centriod.erase(sliding_centriod.begin());
                } // should now return a frame size of 6

                sliding_sum.push_back(sum); // insert sum at the back
                sliding_centriod.push_back(centriod); // insert centriods at the back


                sum.clear();
                score.clear();
                fall_score.clear();
                //cc.clear();
                //range_image.clear();
                remove_me.clear(); //clear the index array and prep for the next one
                cluster_indices.clear(); //empty indices pointer
                //cloud_cluster.clear(); //test
                //centriod.clear(); //test
                border_points.points.clear(); //empty border point pointer
                viewersPtr[i]->spinOnce (125);
            }
        }
    }
    
    // If view closed, remove device
    HPS3D_RemoveDevice(&handle);
    printf("Removing device before closing \n");
    return 0;
}
