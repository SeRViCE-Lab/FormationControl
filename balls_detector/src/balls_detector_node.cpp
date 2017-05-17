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

using namespace cv;
using namespace std;

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
        // cv::imshow(windowName, color);
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
    // This is default parameters for SimpleBlobDetector
    pDefaultBLOB.thresholdStep = 10;
    pDefaultBLOB.minThreshold = 10;
    pDefaultBLOB.maxThreshold = 220;
    pDefaultBLOB.minRepeatability = 2;
    pDefaultBLOB.minDistBetweenBlobs = 10;
    pDefaultBLOB.filterByColor = false;
    pDefaultBLOB.blobColor = 0;
    pDefaultBLOB.filterByArea = false;
    pDefaultBLOB.minArea = 25;
    pDefaultBLOB.maxArea = 1000;
    pDefaultBLOB.filterByCircularity = false;
    pDefaultBLOB.minCircularity = (float)0.9;
    pDefaultBLOB.maxCircularity = (float)1e5;
    pDefaultBLOB.filterByInertia = false;
    pDefaultBLOB.minInertiaRatio = 0.5f;
    pDefaultBLOB.maxInertiaRatio = (float)1e2;
    pDefaultBLOB.filterByConvexity = false;
    pDefaultBLOB.minConvexity = 0.95f;
    pDefaultBLOB.maxConvexity = (float)1e2;
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

    // Param for second BLOB detector we want area between 500 and 2900 pixels
    typeDesc.push_back("BLOB");
    pBLOB.push_back(pDefaultBLOB);
    pBLOB.back().filterByArea = true;
    pBLOB.back().minArea = 2800;
    pBLOB.back().maxArea = 5500;
    // Param for third BLOB detector we want only circular object
    // typeDesc.push_back("BLOB");
    // Param for Fourth BLOB detector we want ratio inertia
    // typeDesc.push_back("BLOB");
    // pBLOB.push_back(pDefaultBLOB);
    // pBLOB.back().filterByInertia = true;
    // pBLOB.back().minInertiaRatio = 0.7;
    // pBLOB.back().maxInertiaRatio = (float)1.0;
    // Param for fifth BLOB detector we want ratio inertia
    // typeDesc.push_back("BLOB");
    // pBLOB.push_back(pDefaultBLOB);
    // pBLOB.back().filterByConvexity = true;
    // pBLOB.back().minConvexity = 0.6;
    // pBLOB.back().maxConvexity = (float)0.9;
    // Param for six BLOB detector we want blob with gravity center color equal to 0 bug #4321 must be fixed
    // typeDesc.push_back("BLOB");
    // pBLOB.push_back(pDefaultBLOB);
    // pBLOB.back().filterByColor = true;
    // pBLOB.back().blobColor = 0;

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
                int i = 0;
                for (vector<KeyPoint>::iterator k = keyImg.begin(); k != keyImg.end(); ++k, ++i)
                    circle(result, k->pt, (int)k->size, palette[i % 65536]);
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

  Receiver r;
  r.run();

  if(!ros::ok())
  {
    return 0;
  }

  ros::shutdown();
}
