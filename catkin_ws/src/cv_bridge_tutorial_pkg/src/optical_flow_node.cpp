#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <ros/console.h>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Raw Image window";
static const std::string OPENCV_WINDOW_1 = "Optical Flow";

class Optical_Flow
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv::Mat prev_src_gray;
  bool needToInit;
  vector<Point2f> points[2];
  int count;
  
public:
  Optical_Flow()
    : it_(nh_)
  {
    // initialize prevImg to zeros
    prev_src_gray = cv::Mat(1, 1, CV_64F, 0.0);
    needToInit = true;
    count = 1;
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, 
      &Optical_Flow::imageCb, this);
    image_pub_ = it_.advertise("/optical_flow/raw_image", 1);
    cv::namedWindow(OPENCV_WINDOW);

  }

  ~Optical_Flow()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr;
    namespace enc = sensor_msgs::image_encodings;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 400 && cv_ptr->image.cols > 600){

	add_optical_flow(cv_ptr->image);
    	image_pub_.publish(cv_ptr->toImageMsg());

	}
  }
  void add_optical_flow(cv::Mat img)
  {
	const int MAX_COUNT = 500;
	const int RESET_COUNT = 250;
	
	if (count % RESET_COUNT == 0) {
		count = 0;
		needToInit = true;	
	}
	cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);
	cv::Size subPixWinSize(10,10), winSize(31,31);

   	cv::Mat src, src_gray;
	cv::Mat dst, detected_edges;	
	
	img.copyTo(src);
		
	cv::cvtColor( img, src_gray, CV_BGR2GRAY );
	
	if ( needToInit ) {
		//initialize 
		cv::goodFeaturesToTrack(src_gray, points[1], MAX_COUNT, 0.01, 10, cv::Mat(), 3, 0, 0.04);
		cv::cornerSubPix(src_gray, points[1], subPixWinSize, cv::Size(-1,-1), termcrit);
		needToInit = false;
		//ROS_ERROR("Initialized");
		
	} else if( !points[0].empty() ) {
		
		vector<uchar> status;
		vector<float> err;
		

		if( prev_src_gray.empty() ) {
			src_gray.copyTo(prev_src_gray);	
		}
		
		cv::calcOpticalFlowPyrLK(prev_src_gray, src_gray, points[0], points[1], status, err, winSize,
					3, termcrit, 0, 0.001);
				
		size_t i, k;
		for( i = k = 0; i < points[1].size(); i++ ) {
			if( !status[i] )
				continue;
			points[1][k++] = points[1][i];
			circle( img, points[1][i], 3, cv::Scalar(0,255,0), -1, 8);
			circle( img, points[0][i], 3, cv::Scalar(255,0,0), -1, 8);
		}
		
		points[1].resize(k);
		
	  	//dst = cv::Scalar::all(0);
	  	//img.copyTo( dst, detected_edges);
		//dst.copyTo(img);

			
	}
	
	img.copyTo(dst);
	
	cv::imshow(OPENCV_WINDOW, src);
	cv::imshow(OPENCV_WINDOW_1, dst);
	cv::waitKey(3);	
	
	std::swap(points[1], points[0]);
	
	cv::swap(prev_src_gray, src_gray);
	count++;
  }	
 
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Optical_Flow");
  Optical_Flow ic;
  ros::spin();
  return 0;
}
