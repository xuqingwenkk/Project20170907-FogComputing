#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "fog_msgs/Descriptor.h"
#include "cam_model_omni.h"
#include "BriefExtractor.h"
using namespace OmniSLAM;
using namespace std;

ros::Publisher pub;
void imageCallback(const sensor_msgs::ImageConstPtr& image){
    ROS_INFO("I receive image");
    cv::Mat img= cv_bridge::toCvShare(image, "bgr8")->image;
    CamModelGeneral cam_model;
    int mn_min_x = 0;
    int mn_max_x = cam_model.GetWidth();
    int mn_min_y = 0;
    int mn_max_y = cam_model.GetHeight();
    BriefExtractor* brief_extractor;
    std::vector<cv::KeyPoint> key_points_temp;
    cv::Mat m_descriptors;
    cv::Mat m_descriptor_masks;
    /*
    BriefExtractor(int nfeatures = 1000,
            float scale_factor = 1.2,
            int nlevels = 8,
            int edge_threshold = 25,
            int first_level = 0,
            int score_type = HARRIS_SCORE,
            int patch_size = 32,
            int fast_threshold = 20,
            bool use_agast = false,
            int fast_agast_type = 2,
            bool do_dBrief = false,
            bool learn_masks = false,
            int desc_size = 32);
    */
    brief_extractor = new BriefExtractor(400, 1.2, 8, 25, 0, 0, 32, 20, false, 2, false, false, 32);
    (*brief_extractor)(img,
                       cam_model.GetMirrorMask(0),
                       key_points_temp,
                       cam_model,
                       m_descriptors,
                       m_descriptor_masks);
//    for(size_t i = 0; i < key_points_temp.size(); ++i){
//        cout << "keypoint " << i << " "
//             << key_points_temp.at(i).angle << " " << key_points_temp.at(i).size << " "
//                << key_points_temp.at(i).class_id << " " << key_points_temp.at(i).octave << " "
//                << key_points_temp.at(i).pt.x << " " << key_points_temp.at(i).pt.y << " "
//                << key_points_temp.at(i).response;
//    }
//    fog_msgs::Descriptor descriptor;
//    descriptor.descriptors = *(cv_bridge::CvImage(std_msgs::Header(), "bgr8", m_descriptors).toImageMsg());
//    descriptor.masks = *(cv_bridge::CvImage(std_msgs::Header(), "bgr8", m_descriptor_masks).toImageMsg());
//    pub.publish(descriptor);
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "feature_extractor");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/image", 1, imageCallback);
    pub = nh.advertise<fog_msgs::Descriptor>("features", 1);
    ros::spin();
}