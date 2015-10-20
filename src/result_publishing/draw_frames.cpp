#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <fstream>
class FrameDrawer 
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_;
  image_transport::Publisher pub_;

ros::NodeHandle nh_2;
 // image_transport::ImageTransport it_2;
  ros::Subscriber sub_2;



  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  std::vector<std::string> frame_ids_;
  CvFont font_;
double x;
double y;
 //cv::Point pos;

public:
  std::ofstream myfile;  
  FrameDrawer(const std::vector<std::string>& frame_ids)
    : it_(nh_), frame_ids_(frame_ids)
  {
    std::string image_topic = nh_.resolveName("camera/image_raw");
    sub_ = it_.subscribeCamera(image_topic, 1, &FrameDrawer::imageCb, this);
    pub_ = it_.advertise("image_out", 1);

    std::string result_topic = nh_2.resolveName("/kinematic_calibration/result_data");
    sub_2 = nh_2.subscribe(result_topic, 1, &FrameDrawer::resultCb, this);
    cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
    myfile.open ("/home/kurt/catkin_ws/result_calibrated.txt");
  }

void resultCb(const std_msgs::String markerData)
  {
	std::cout<<markerData<<std::endl;//" "<<markerData[1]<<"\t"<<pos[0]<<" "<<pos[1]<<std::endl;
         myfile << x <<" "<<y<<"\n";
}
  
void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg
		)
  {

    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    try {
      input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("[draw_frames] Failed to convert image");
      return;
    }

    cam_model_.fromCameraInfo(info_msg);

    BOOST_FOREACH(const std::string& frame_id, frame_ids_) {
      tf::StampedTransform transform;
      try {
        ros::Time acquisition_time = info_msg->header.stamp;
        ros::Duration timeout(4.0 / 30.0);
        tf_listener_.waitForTransform("CameraBottom_frame", frame_id,
                                      acquisition_time, timeout);
        tf_listener_.lookupTransform("CameraBottom_frame", frame_id,
                                     acquisition_time, transform);
      }
      catch (tf::TransformException& ex) {
        ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        return;
      }

      tf::Point pt = transform.getOrigin();
      cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
      cv::Point2d uv;
      uv = cam_model_.project3dToPixel(pt_cv);
      this->x=uv.x;
      this->y=uv.y;
     // pos[0]=uv.x;pos[1]=uv.y;
     // std::cout<<uv.x <<" "<<uv.y<<" "<<markerData[0]<< " "<<markerData[1]<<std::endl;
      static const int RADIUS = 3;
      cv::circle(image, uv, RADIUS, CV_RGB(255,0,0), -1);
      CvSize text_size;
      int baseline;
	
      cvGetTextSize(frame_id.c_str(), &font_, &text_size, &baseline);
      CvPoint origin = cvPoint(uv.x - text_size.width / 2,
                               uv.y - RADIUS - baseline - 3);
      //std::cout<<origin.y <<" "<<origin.x <<std::endl;
    cv:putText(image, frame_id.c_str(), origin, cv::FONT_HERSHEY_SIMPLEX, 12, CV_RGB(255,0,0));
    }

    pub_.publish(input_bridge->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "draw_frames");
  std::vector<std::string> frame_ids(argv + 1, argv + argc);
  FrameDrawer drawer(frame_ids);
//keyboardEventHandler* keh = new keyboardEventHandler();
// this.getEventHandlerList().push_front(keh); 
/*while (ros::ok())
{
char k=cvWaitKey(0.5);
if(k == 'C' ||k == 'c')
	std::cout<<"OMG"<<std::endl;*/
  ros::spin();
//}
drawer.myfile.close();
/*if (KEYPRESSED('M')){ 
std::cout<<"Preeessed";
Sleep(50);*/
 
}

