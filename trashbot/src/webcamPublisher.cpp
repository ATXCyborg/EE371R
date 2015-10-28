#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> //for converting the command line parameter to integer
#include <cstdio>
#include <stdlib.h>

int main(int argc, char** argv){
    int rightAddress;
    int leftAddress;

    //Check if the video source has been passed as a parameter
    if((argv[1] == NULL) && (argv[2] == NULL)){
        return 1;
    }

    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image",1);

    //Convert the passed command line parameter as index for the video device to an integer
//    printf("Converting passed parameters to video device address. TEST!\n");
//    std::istringstream video_sourcedCmd(argv[1]); //this is the faulting line!
//    int video_source;
//    //Check if it is indeed a number
//    if(!(video_sourcedCmd >> video_source)) return 1;
    leftAddress = atoi(argv[1]);
    rightAddress = atoi(argv[2]);
    cv::VideoCapture cap(leftAddress);
    //Check if video device canbe opened with the given index
    if(!cap.isOpened()) return 1;
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(60);
    while(nh.ok()){
        cap >> frame;
        //Check if grabbed frame is actually full with some content
        if(!frame.empty()){
            msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", frame).toImageMsg();
            pub.publish(msg);
            cv::waitKey(1);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    printf("Something happened. I'm exiting!\n");

    return 0;
}
