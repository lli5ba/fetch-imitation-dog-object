#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

// should probably do this in class, but in a hurry
enum demo { demoORIG, demoHSV, demoFLOW, demoFLOWBW, demoRED };
demo show = demoFLOW;

static const std::string OPENCV_WINDOW = "Raw Image window";
static const std::string OPENCV_WINDOW_1 = "Dense Optical Flow";

// From OPEN CV Examples, tvl1_optical_flow.cpp
inline bool isFlowCorrect(cv::Point2f u)
{
    return !cvIsNaN(u.x) && !cvIsNaN(u.y) && fabs(u.x) < 1e9 && fabs(u.y) < 1e9;
}

// From OPEN CV Examples, tvl1_optical_flow.cpp
static cv::Vec3b computeColor(float fx, float fy)
{
    static bool first = true;

    // relative lengths of color transitions:
    // these are chosen based on perceptual similarity
    // (e.g. one can distinguish more shades between red and yellow
    //  than between yellow and green)
    const int RY = 15;
    const int YG = 6;
    const int GC = 4;
    const int CB = 11;
    const int BM = 13;
    const int MR = 6;
    const int NCOLS = RY + YG + GC + CB + BM + MR;
    static cv::Vec3i colorWheel[NCOLS];

    if (first)
    {
        int k = 0;

        for (int i = 0; i < RY; ++i, ++k) {
	  colorWheel[k] = cv::Vec3i(255, 255 * i / RY, 0);
	}

        for (int i = 0; i < YG; ++i, ++k) {
	  colorWheel[k] = cv::Vec3i(255 - 255 * i / YG, 255, 0);
	}

        for (int i = 0; i < GC; ++i, ++k) {
	  colorWheel[k] = cv::Vec3i(0, 255, 255 * i / GC);
	}

        for (int i = 0; i < CB; ++i, ++k) {
	  colorWheel[k] = cv::Vec3i(0, 255 - 255 * i / CB, 255);
	}

        for (int i = 0; i < BM; ++i, ++k) {
	  colorWheel[k] = cv::Vec3i(255 * i / BM, 0, 255);
	}

        for (int i = 0; i < MR; ++i, ++k) {
	  colorWheel[k] = cv::Vec3i(255, 0, 255 - 255 * i / MR);
	}

        first = false;
    }

    const float rad = std::sqrt(fx * fx + fy * fy);
    const float a = atan2(-fy, -fx) / (float)CV_PI;
    //@@@const float a = atan2(fy, fx) + (float)CV_PI;

    const float fk = (a + 1.0f) / 2.0f * (NCOLS - 1);
    const int k0 = static_cast<int>(fk);
    const int k1 = (k0 + 1) % NCOLS;
    const float f = fk - k0;

    cv::Vec3b pix;

    for (int b = 0; b < 3; b++) {
      const float col0 = colorWheel[k0][b] / 255.f;
      const float col1 = colorWheel[k1][b] / 255.f;

      float col = (1 - f) * col0 + f * col1;
      
      if (rad <= 1) {
	col = 1 - rad * (1 - col); // increase saturation with radius
      } else {
	col *= .75; // out of range
      }

      pix[2 - b] = static_cast<uchar>(255.f * col);
    }

    return pix;
}

// based on opencv python example, sample/python/opt_flow.py
static cv::Vec3b computeHSV(float fx, float fy)
{
    const float rad = std::sqrt(fx * fx + fy * fy);
    const float a = atan2(fy, fx) + (float)CV_PI;  // change output to 0 to 2*pi
    const int sensitivity = 5;	// make high in order to be more sensitive to motion
    cv::Vec3b pix;

    pix[0] = static_cast<int>(a*(255/(2*(float)CV_PI)));		// Hue
    pix[1] = 255;		                                        // Saturation (full)
    pix[2] = std::min(static_cast<int>(rad*sensitivity),255);		// Value

    return pix;
}

static cv::Vec3b computeBGR(float fx, float fy)
{
    const float mag = std::sqrt(fx * fx + fy * fy);
    //const float a = atan2(fy, fx) + (float)CV_PI;  // change output to 0 to 2*pi
    const float threshold = 6.5; // threshold that motion magnitude must be above to see
    const int sensitivity = 12;	 // scalar to magnitude
    int mag_int;
    cv::Vec3b pix;

    if (mag < threshold) {
	mag_int = 0;
    } else {
	mag_int = static_cast<int>(mag * sensitivity);
    }

    pix[0] = 0;	    	                // Blue
    pix[1] = 0;		                // Green
    pix[2] = std::min(mag_int,255);	// Red

    return pix;
}


/*
def draw_flow(img, flow, step=16):
    h, w = img.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1).astype(int)
    fx, fy = flow[y,x].T
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.polylines(vis, lines, 0, (0, 255, 0))
    for (x1, y1), (x2, y2) in lines:
        cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    return vis
*/
class Optical_Flow
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv::Mat prevgray;

public:
  Optical_Flow()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    // /* USB Cam: */ image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &Optical_Flow::imageCb, this);
    /* Asus xton */ image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &Optical_Flow::imageCb, this);
    image_pub_ = it_.advertise("/dense_optical_flow/raw_image", 1);
    cv::namedWindow(OPENCV_WINDOW);

  }

  ~Optical_Flow()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  // Callback for Image (called each time receive a frame ?)
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

    // Perform optical flow algorithm on the image
    if (cv_ptr->image.rows > 400 && cv_ptr->image.cols > 600){

	optical_flow(cv_ptr->image);
    	image_pub_.publish(cv_ptr->toImageMsg());

	}
  }

  void optical_flow(cv::Mat img)
  {

        cv::Mat src, src_gray;
	cv::Mat dst, detected_edges;
	cv::Mat flow;

	img.copyTo(src);

	cv::cvtColor( img, src_gray, CV_BGR2GRAY );
	
	if(prevgray.empty())
	  src_gray.copyTo( prevgray );
	
	cv::calcOpticalFlowFarneback(prevgray, src_gray, flow, 
				     /* pyr_scale */ 0.5, 
				     /* levels */ 2, 
				     /* winsize */ 15, 
				     /* iterations */ 2,  /* was: 3 */ 
				     /* poly_n */ 5, 
				     /* poly_sigma */ 1.5, 
				     /* flags */ 0);
  	src_gray.copyTo( prevgray );

	// last parameter is a threshold - make lower to be more sensitive to movement
	drawOpticalFlow(flow, src_gray, dst, 100);

  	//dst = cv::Scalar::all(0);
  	//img.copyTo( dst, detected_edges);
	dst.copyTo(img);

    	cv::imshow(OPENCV_WINDOW, src);
    	cv::imshow(OPENCV_WINDOW_1, dst);
    	char c = (char) cv::waitKey(3);

	// Press ESC to quit
        if( c == 27 )
	    ros::shutdown();

	// Press '1' key to toggle demos
        if( c == '1' ) {
	    if (show == demoFLOW) {
		show = demoFLOWBW;
	    } else if (show == demoFLOWBW) {
		//skip HSV: show = demoHSV;
		show = demoRED;
	    } else if (show == demoHSV) {
		show = demoRED;
	    } else if (show == demoRED) {
		show = demoFLOW;
	    }
        }


  }	

  // From OPEN CV Examples, tvl1_optical_flow.cpp
  void drawOpticalFlow(const cv::Mat_<cv::Point2f>& flow, const cv::Mat img, cv::Mat& dst, float maxmotion = -1)
  {
    cv::Mat hsv;
    
    if (show == demoORIG || show == demoHSV || show == demoRED || show == demoFLOW) {
	dst.create(flow.size(), CV_8UC3);
	dst.setTo(cv::Scalar::all(0));
	dst.copyTo(hsv);
    }

    if (show == demoFLOWBW) {
	//@@@img.copyTo(dst);
	cv::cvtColor( img, dst, CV_GRAY2BGR );
    }

    // determine motion range:
    float maxrad = maxmotion;

    // set maxmotion to -1 to have algorithm find the maximum flow value
    if (maxmotion <= 0) {
      maxrad = 1;
      for (int y = 0; y < flow.rows; ++y) {
	for (int x = 0; x < flow.cols; ++x) {
	  cv::Point2f u = flow(y, x);

	  if (!isFlowCorrect(u))
	    continue;

	  maxrad = std::max(maxrad, std::sqrt(u.x * u.x + u.y * u.y));
	}
      }
    }

    int step = 16; // for show == demoFLOW

    for (int y = 0; y < flow.rows; ++y) {
      for (int x = 0; x < flow.cols; ++x) {
	cv::Point2f u = flow(y, x);

	if (isFlowCorrect(u)) {
	    if (show == demoORIG) {
		dst.at<cv::Vec3b>(y, x) = computeColor(u.x / maxrad, u.y / maxrad);
	    }

	    if (show == demoHSV) {
		hsv.at<cv::Vec3b>(y, x) = computeHSV(u.x, u.y);
	    }

	    if (show == demoRED) {
		dst.at<cv::Vec3b>(y, x) = computeBGR(u.x, u.y);
	    }

	    if ((show == demoFLOW || show == demoFLOWBW) && (y%step == 0) && (x%step == 0)) {
		cv::line(dst, cv::Point(x, y), cv::Point(x+u.x, y+u.y), cv::Scalar(0, 255, 0), 1);
		cv::circle(dst, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), -2);
	    }
		
	}

      }
    }

    if (show == demoHSV) {
	// Convert HSV image to BGR before finish
	cv::cvtColor( hsv, dst, CV_HSV2BGR );
    }

  }
 
};

int main(int argc, char** argv)
{
    printf("\n");
    printf("Press 'ESC' in CV window to quit.\n");
    printf("Press '1' in CV window to toggle demos.\n");

  ros::init(argc, argv, "Optical_Flow");
  Optical_Flow ic;
  ros::spin();
  return 0;
}
