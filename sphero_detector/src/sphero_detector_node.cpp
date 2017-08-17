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
#include "std_msgs/Float64MultiArray.h"


using namespace cv;
using namespace std;

class Receiver
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher locs_pub_;

  const unsigned long hardware_threads;
  Mat color;
  std::vector<std::thread> threads;
  std::thread imageDispThread;
  bool running, updateImage;
  mutable std::string windowName;
  ros::AsyncSpinner spinner;

  std::mutex mutex;
  std::string camera;

  // detection parameters
  int threshold_step, min_threshold, max_threshold, min_repeatability,
      min_dist_between_blobs, blob_color, min_area, max_area, numRob;

  bool filter_by_color, filter_by_area, filter_by_convexity,
        filter_by_circularity, filter_by_inertia, done;

  double min_circularity, max_circularity, min_inertia_ratio,
         max_inertia_ratio, min_convexity, max_convexity;

public:
  Receiver(ros::NodeHandle nh)
  : nh_(nh), it_(nh_), hardware_threads(std::thread::hardware_concurrency()),
   spinner(hardware_threads/2), updateImage(false)
  {
    windowName = "cam_rgb";
    image_sub_ = it_.subscribe("/cam/image/rgb", 1, &Receiver::imageCallback, this);
    locs_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/locs/detected", 1000);

    imageDispThread = std::thread(&Receiver::imageDisp, this);
    getParams();
  }

  ~Receiver()
  {
    destroyAllWindows();
  }

  Receiver(Receiver const&) = delete;
  Receiver& operator=(Receiver const&) = delete;

  void run() {
    begin();
    end();
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
    if (imageDispThread.joinable())    {
      imageDispThread.join();
    }
  }

  void end()  {
    spinner.stop();
    running = false;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    Mat color;
    std::lock_guard<std::mutex> lock(mutex);
    getImage(msg, color);
    this->color = color;
    updateImage = true;
  }

  void getImage(const sensor_msgs::ImageConstPtr msgImage, Mat &image) const
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
    cv::Mat color, blob_circs(640, 480, CV_8UC3);
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, 640, 480) ;

    for(; running && ros::ok() ;)
    {
      if(updateImage)
      {
        std::lock_guard<std::mutex> lock(mutex);
        color = this->color;
        updateImage = false;

        detectBlobs(std::move(color), std::move(blob_circs));
        cv::imshow(windowName, blob_circs);
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
    pDefaultBLOB.maxConvexity =max_convexity;
    // Descriptor array for BLOB
    vector<String> typeDesc;
    // Param array for BLOB
    vector<SimpleBlobDetector::Params> pBLOB;
    vector<SimpleBlobDetector::Params>::iterator itBLOB;
    // Color palette
    vector< Vec3b >  palette;
    for (int i = 0; i<65536; i++)
    {
        palette.push_back(Vec3b((uchar)rand(), (uchar)rand(), (uchar)rand()));
    }

    // Param for the BLOB detector
    typeDesc.push_back("BLOB");
    pBLOB.push_back(pDefaultBLOB);

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
            Mat     desc; //, result(img.rows, img.cols, CV_8UC3);
            if (b.dynamicCast<SimpleBlobDetector>() != NULL)
            {
                Ptr<SimpleBlobDetector> sbd = b.dynamicCast<SimpleBlobDetector>();
                sbd->detect(img, keyImg, Mat());
                drawKeypoints(img, keyImg, result);
                std_msgs::Float64MultiArray locs;
                locs.data.clear();

                int i = 0;
                for (vector<KeyPoint>::iterator k = keyImg.begin(); k != keyImg.end(); ++k, ++i)
                {
                  cout << keyImg[i].pt.x<<";"<<keyImg[i].pt.y<<endl;
                  locs.data.push_back(keyImg[i].pt.x);
                  locs.data.push_back(keyImg[i].pt.y);

                  circle(result, k->pt, (int)k->size, Scalar(0, 255, 255), 1);
                }
                locs_pub_.publish(locs);
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

  Receiver rcvr(nh);
  rcvr.run();

  if(!ros::ok())
  {
    return 0;
  }

  ros::shutdown();
}
