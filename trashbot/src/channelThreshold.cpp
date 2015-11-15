#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <cstdlib>

using namespace std;

class channelThreshold
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  int thresholdMax[3] = {255,255,255};
  int thresholdMin[3] = {0,0,0};

public:
  channelThreshold(int* max, int* min): it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("video/raw", 1, &channelThreshold::imageCb, this);
    image_pub_ = it_.advertise("video/threshed", 1);

  }

  ~channelThreshold()
  {
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr image;
    try
    {
      image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Output modified video stream
    image_pub_.publish(image->toImageMsg());
  }
};

int main(int argc, char** argv)
{

  //check if values to threshold channel
  if((argv[1] == NULL)){
        return 1;
    }

  int min[3] = {atoi(argv[1]), atoi(argv[2]), atoi(argv[3])};
  int max[3] = {atoi(argv[4]), atoi(argv[5]), atoi(argv[6])};

  ros::init(argc, argv, "image2HSVConverter");
  channelThreshold ct(min, max);
  ros::spin();
  return 0;
}
