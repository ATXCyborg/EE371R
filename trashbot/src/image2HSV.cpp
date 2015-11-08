#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

class image2HSVConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  image2HSVConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("camera/image", 1, &image2HSVConverter::imageCb, this);
    image_pub_ = it_.advertise("image2hsv_converter/hsv_video", 1);

  }

  ~image2HSVConverter()
  {
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr color;
    try
    {
      color = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //convert image from BGR to HSV space
    cv::cvtColor(color->image, color->image, cv::COLOR_BGR2HSV);

    // Output modified video stream
    image_pub_.publish(color->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image2HSVConverter");
  image2HSVConverter ic;
  ros::spin();
  return 0;
}
