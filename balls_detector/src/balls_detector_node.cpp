#include <thread>
#include <vector>
#include <mutex>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

class Receiver
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  const unsigned long hardware_threads;
  cv::Mat color;
  std::vector<std::thread> threads;
  std::thread imageDispThread;
  bool running, updateImage;
  const std::string windowName;
  ros::AsyncSpinner spinner;

  std::mutex mutex;
public:
  Receiver()
  : it_(nh_), hardware_threads(std::thread::hardware_concurrency()),
  windowName("kinect_rgb"), spinner(hardware_threads/2), updateImage(false)
  {
    image_sub_ = it_.subscribe("/kinect/image/rgb", 1, &Receiver::imageCallback, this);
    imageDispThread = std::thread(&Receiver::imageDisp, this);
  }

  ~Receiver()
  {
    cv::destroyWindow("kinect_rgb");
  }

  Receiver(Receiver const&) =delete;
  Receiver& operator=(Receiver const&) = delete;

  void run() {
    begin();
    end();
  }

private:
  void begin()
  {
    if(spinner.canStart())
    {
      spinner.start();
    }
    running = true;
    while(!updateImage)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    //spawn the threads
    if (imageDispThread.joinable())
    {
      imageDispThread.join();
    }
    ROS_INFO("received image");
  }

  void end()
  {
    spinner.stop();
    running = false;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv::Mat color;
    std::lock_guard<std::mutex> lock(mutex);
    getImage(msg, color);
    this->color = color;
    updateImage = true;
  }

  void getImage(const sensor_msgs::ImageConstPtr msgImage, cv::Mat &image) const
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(msgImage, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv_ptr->image.copyTo(image);
  }

  void imageDisp()
  {
    cv::Mat color;
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, 640, 480) ;

    for(; running && ros::ok() ;)
    {
      if(updateImage)
      {
        std::lock_guard<std::mutex> lock(mutex);
        color = this->color;
        updateImage = false;

        detectBlobs(std::move(color));
        cv::imshow(windowName, color);
      }

      int key = cv::waitKey(1);
      switch(key & 0xFF)
      {
        case 27:
          break;
        case 'q':
          running = false;
          break;
      }
    }
    cv::destroyAllWindows();
    cv::waitKey(5);
  }

  void detectBlobs(cv::Mat && color_)
  {
    // Setup SimpleBlobDetector parameters.
    cv::SimpleBlobDetector::Params params;

    // Change thresholds
    // params.minThreshold = 10;
    // params.maxThreshold = 100;

    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 100;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.10;

    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.65;

    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.6;

    std::vector<cv::KeyPoint> keypoints;
    #if CV_MAJOR_VERSION < 3
      cv::SimpleBlobDetector detector(params);
      detector.detect( color, keypoints);
    #else
      cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
      detector->detect( color_, keypoints);
    #endif

    cv::Mat mat_keypoints_;
    drawKeypoints(color_, keypoints, mat_keypoints_, cv::Scalar(255, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sphero_detector_node");
  ros::NodeHandle nh;

  Receiver r;
  r.run();

  if(!ros::ok())
  {
    return 0;
  }

  ros::shutdown();
}
