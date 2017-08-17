#include <thread>
#include <string>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <std_msgs/ColorRGBA.h>
#include "std_msgs/Float64MultiArray.h"


using namespace cv;
using namespace std;

class Receiver
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher tags_pub_;

  Mat frame;
  mutable string windowName;

  thread taggersThread, publishersThread;

  std_msgs::Float64MultiArray locs_tagged;

  // detection parameters
  int threshold_step, min_threshold, max_threshold, min_repeatability,
      min_dist_between_blobs, blob_color, min_area, max_area, numRob;

  bool filter_by_color, filter_by_area, filter_by_convexity,
        filter_by_circularity, filter_by_inertia, done, running;

  double min_circularity, max_circularity, min_inertia_ratio,
         max_inertia_ratio, min_convexity, max_convexity;

public:
  Receiver(const bool& done, ros::NodeHandle nh)
  : done(false), nh_(nh), running(true), it_(nh_)
  {
    nh_.getParam("/tagger/Rob_Params/num_rob", numRob);
    windowName = "cam_rgb_tagged";
    image_sub_ = it_.subscribe("/cam/image/rgb", 1, &Receiver::imageCallback, this);
    tags_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/locs/tagged", 10);
  }

  // delete copy constructors
  Receiver(Receiver const&) = delete;
  Receiver& operator=(Receiver const&) = delete;

  ~Receiver()
  {
    cv::destroyAllWindows();
  }

  void begin()
  {
    getParams();
    taggersThread = std::thread(&Receiver::assign_tags, this);
    publishersThread = std::thread(&Receiver::publish_data, this);
  }

private:

  template <typename T>
  void retrieveParam(std::string param, T value) const
  {
    ros::param::get("/tagger/Blob_Params/" + param, value);
  }

  void getParams()
  {
    retrieveParam("threshold_step", threshold_step);
    retrieveParam("min_threshold", min_threshold);
    retrieveParam("max_threshold", max_threshold);
    retrieveParam("min_repeatability", min_repeatability);
    retrieveParam("min_dist_between_blobs", min_dist_between_blobs);
    retrieveParam("filter_by_color", filter_by_color);
    retrieveParam("blob_color", blob_color);
    retrieveParam("filter_by_area", filter_by_area);
    retrieveParam("max_area", max_area);
    retrieveParam("min_area", min_area);
    retrieveParam("filter_by_circularity", filter_by_circularity);
    retrieveParam("min_circularity", min_circularity);
    retrieveParam("max_circularity", max_circularity);
    retrieveParam("filter_by_inertia", filter_by_inertia);
    retrieveParam("min_inertia_ratio", min_inertia_ratio);
    retrieveParam("max_inertia_ratio", max_inertia_ratio);
    retrieveParam("filter_by_convexity", filter_by_convexity);
    retrieveParam("min_convexity", min_convexity);
    retrieveParam("max_convexity", max_convexity);
  }

  void assign_tags()
  {
    //// Defining LED colors
    std_msgs::ColorRGBA led_off, led_on;
    led_off.r = 0; led_off.g = 0; led_off.b = 0; led_off.a = 0; // robot LEDs off
    led_on.r = 1; led_on.g = 1; led_on.b = 1; led_on.a = 0; // robots LEDs ON

    // Initializing advertising topics for spheros' LED color
    std::string led_color_topic [numRob];
    ros::Publisher color_pub_array [numRob];
    for (size_t num_rob = 0; num_rob < numRob; ++num_rob)
    {
      std::string rob_id = "sph" + std::to_string(num_rob);
      std::string rob_name;
      nh_.getParam("/tagger/Rob_Params/"+rob_id, rob_name);
      std::string topic_name = "/" + rob_name + "/set_color";
      led_color_topic [num_rob] = topic_name;
      color_pub_array [num_rob] = nh_.advertise<std_msgs::ColorRGBA>(led_color_topic[num_rob], 1);
    }

    //// Turn off LEDs
    for (int num_rob = 0; num_rob < numRob; ++num_rob)
    {
      color_pub_array[num_rob].publish(led_off);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Loop through the spheros: turn on one robot, detect it, turn it off, repeat
    for (int num_rob = 0; num_rob < numRob; ++num_rob)
    {
      color_pub_array[num_rob].publish(led_on);

      // Detect robot/blob in frame
      detectBlobs(std::move(frame),std::move(frame));

      // Turn that robot off
      color_pub_array[num_rob].publish(led_off);
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));

      // Turn on LEDs
      color_pub_array[num_rob].publish(led_on);
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    // Publish tagged locations
    tags_pub_.publish(locs_tagged);

    ros::Rate looper(30);
    looper.sleep();

    done = true;
  }

  void publish_data()
  {
    for(; running && ros::ok() ;)
    {
      tags_pub_.publish(locs_tagged);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr frame_ptr;
    try
    {
      frame_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    frame_ptr->image.copyTo(this->frame);
  }

  void imageDisp()
  {
    namedWindow(windowName, WINDOW_NORMAL);
    resizeWindow(windowName, 640, 480);
    if (!frame.empty())
    {
      imshow(windowName, frame);
      int key = waitKey(1);
    }
  }


  static String Legende(SimpleBlobDetector::Params &pAct)
  {
    String s = "";
    if (pAct.filterByArea)
    {
        String inf = static_cast<const ostringstream&>(ostringstream() << pAct.minArea).str();
        String sup = static_cast<const ostringstream&>(ostringstream() << pAct.maxArea).str();
        s = " Area range [" + inf + " to  " + sup + "]";
    }
    if (pAct.filterByCircularity)
    {
        String inf = static_cast<const ostringstream&>(ostringstream() << pAct.minCircularity).str();
        String sup = static_cast<const ostringstream&>(ostringstream() << pAct.maxCircularity).str();
        if (s.length() == 0)
            s = " Circularity range [" + inf + " to  " + sup + "]";
        else
            s += " AND Circularity range [" + inf + " to  " + sup + "]";
    }
    if (pAct.filterByColor)
    {
        String inf = static_cast<const ostringstream&>(ostringstream() << (int)pAct.blobColor).str();
        if (s.length() == 0)
            s = " Blob color " + inf;
        else
            s += " AND Blob color " + inf;
    }
    if (pAct.filterByConvexity)
    {
        String inf = static_cast<const ostringstream&>(ostringstream() << pAct.minConvexity).str();
        String sup = static_cast<const ostringstream&>(ostringstream() << pAct.maxConvexity).str();
        if (s.length() == 0)
            s = " Convexity range[" + inf + " to  " + sup + "]";
        else
            s += " AND  Convexity range[" + inf + " to  " + sup + "]";
    }
    if (pAct.filterByInertia)
    {
        String inf = static_cast<const ostringstream&>(ostringstream() << pAct.minInertiaRatio).str();
        String sup = static_cast<const ostringstream&>(ostringstream() << pAct.maxInertiaRatio).str();
        if (s.length() == 0)
            s = " Inertia ratio range [" + inf + " to  " + sup + "]";
        else
            s += " AND  Inertia ratio range [" + inf + " to  " + sup + "]";
    }
    return s;
  }

  void detectBlobs(cv::Mat && img, cv::Mat && result)
  {
    SimpleBlobDetector::Params pDefaultBLOB;
    // This is the default parameters set for SimpleBlobDetector
    pDefaultBLOB.thresholdStep = threshold_step;
    pDefaultBLOB.minThreshold = min_threshold;
    pDefaultBLOB.maxThreshold = max_threshold;
    pDefaultBLOB.minRepeatability = min_repeatability;
    pDefaultBLOB.minDistBetweenBlobs = min_dist_between_blobs;
    pDefaultBLOB.filterByColor = filter_by_color;
    pDefaultBLOB.blobColor = blob_color;
    pDefaultBLOB.filterByArea = filter_by_area;
    pDefaultBLOB.minArea = min_area;
    pDefaultBLOB.maxArea = max_area;
    pDefaultBLOB.filterByCircularity = filter_by_circularity;
    pDefaultBLOB.minCircularity = min_circularity;
    pDefaultBLOB.maxCircularity = max_circularity;
    pDefaultBLOB.filterByInertia = filter_by_inertia;
    pDefaultBLOB.minInertiaRatio = min_inertia_ratio;
    pDefaultBLOB.maxInertiaRatio = max_inertia_ratio;
    pDefaultBLOB.filterByConvexity = filter_by_convexity;
    pDefaultBLOB.minConvexity = min_convexity;
    pDefaultBLOB.maxConvexity = max_convexity;
    // Descriptor array for BLOB
    vector<String> typeDesc;
    // Param array for BLOB
    vector<SimpleBlobDetector::Params> pBLOB;
    vector<SimpleBlobDetector::Params>::iterator itBLOB;

    // Param for the BLOB detector
    typeDesc.push_back("BLOB");
    pBLOB.push_back(pDefaultBLOB);

    // Detect that robot
    itBLOB = pBLOB.begin();
    vector<double> desMethCmp;
    Ptr<Feature2D> b;
    String label;
    // Descriptor loop
    vector<String>::iterator itDesc;
    for (itDesc = typeDesc.begin(); itDesc != typeDesc.end(); ++itDesc)
    {
      vector<KeyPoint> keyImg1;
      if (*itDesc == "BLOB")
      {
        b = SimpleBlobDetector::create(*itBLOB);
        label = Legende(*itBLOB);
        ++itBLOB;
      }
      try
      {
        // We can detect keypoint with detect method
        vector<KeyPoint>  keyImg;
        vector<Rect>  zone;
        vector<vector <Point> >  region;
        cv::Mat desc; //, result(img.rows, img.cols, CV_8UC3);
        if (b.dynamicCast<SimpleBlobDetector>() != NULL)
        {
          Ptr<SimpleBlobDetector> sbd = b.dynamicCast<SimpleBlobDetector>();
          sbd->detect(img, keyImg, Mat());
          drawKeypoints(img, keyImg, result);

          int i = 0;
          float x = 0;
          float y = 0;
          for (vector<KeyPoint>::iterator k = keyImg.begin(); k != keyImg.end(); ++k, ++i)
          {
            x += keyImg[i].pt.x;
            y += keyImg[i].pt.y;

            circle(result, k->pt, (int)k->size, Scalar(0, 255, 255), 1);

          }
          // avrg the centers
          x = x/(i);
          y = y/(i);
          // store center
          this->locs_tagged.data.push_back(x);
          this->locs_tagged.data.push_back(y);
        }
      }
      catch (Exception& e)
      {
        cout << "Feature : " << *itDesc << "\n";
        cout << e.msg << endl;
      }
    }
  }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sphero_detector_node");
    ros::NodeHandle nh;

    bool done (false);

  Receiver r(done, nh);

  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  if (!done && ros::ok())
  {
      r.begin();
  }

  ros::spin();

  return EXIT_SUCCESS;
}
