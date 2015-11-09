#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

class HSVViewer
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  HSVViewer()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("video/channel3", 1, &HSVViewer::imageCb, this);

  }

  ~HSVViewer()
  {
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr frame;
    std::vector<cv::Mat> channels;
    try
    {
      frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::split(frame->image, channels);

    cv::imshow("Channel 1", channels[0]);
    cv::imshow("Channel 2", channels[1]);
    cv::imshow("Channel 3", channels[2]);
    cv::waitKey(30);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "HSVViewer");
  HSVViewer ic;
  ros::spin();
  return 0;
}
