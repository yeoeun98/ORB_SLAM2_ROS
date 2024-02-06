#include "MonoNode.h"

#include <ctime>
#include <sys/time.h>
#include <chrono>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);

    MonoNode node (ORB_SLAM2::System::MONOCULAR, node_handle, image_transport);

    node.Init();

    ros::spin();

    ros::shutdown();

    return 0;
}


MonoNode::MonoNode (ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : Node (sensor, node_handle, image_transport) {
  image_subscriber = image_transport.subscribe ("/camera/image_raw", 1, &MonoNode::ImageCallback, this);
  // image_subscriber = image_transport.subscribe ("/image_2", 1, &MonoNode::ImageCallback, this);
  camera_info_topic_ = "/camera/camera_info";
}


MonoNode::~MonoNode () {
}


void MonoNode::ImageCallback (const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr cv_in_ptr;
  try {
      cv_in_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msg->header.stamp;

  // cv::Rect bounds(192, 108, 1536, 864);
  // cv::Mat img_rect = cv_in_ptr->image(bounds);
  // cv::Mat img;
  // cv::resize(cv_in_ptr->image, img, cv::Size(1344, 756));
  orb_slam_->TrackMonocular(cv_in_ptr->image,cv_in_ptr->header.stamp.toSec());
  // std::cout<<"#####################################################"<<std::endl;
  // std::cout<<"[ImageCallback] Timestamp : "<<std::fixed<<cv_in_ptr->header.stamp.toSec()<<std::endl;

  Update ();

  // yeoh, make time txt file
  // bool first_case = false;
  // string filePath = "/home/swm/log/orb2_time.txt";
  // ofstream PoseWriteFile;
  // PoseWriteFile.open(filePath.data(), std:: ios::app);

  // if(PoseWriteFile.is_open()){
  //   // map coordinate to camera coordinate
  //   PoseWriteFile<<img_in_time<<"\n";
  // }
}
