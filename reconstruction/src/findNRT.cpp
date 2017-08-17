#include <vector>
#include <string>
#include <iostream>
#include <thread>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/Float64MultiArray.h"
#include <image_transport/image_transport.h>
// #include "/usr/include/eigen3/Eigen/Dense"
#include <Eigen/Dense>

using namespace cv;
using namespace std;
using namespace Eigen;

class FindNRT
{
private:
  // node handle, image, and video variables
  ros::NodeHandle nh_;
  int cam_num, img_width_, img_hight_;
  std::string window_name_;
  VideoCapture cap_;
  Mat frame_;

  // variables for board parameters
  double sqr_size_;
  int row_corners_, col_corners_;

  // variables for camera parameters
  int cam_mat_row_, cam_mat_col_;
  int dist_coef_row_, dist_coef_col_;

  // variables for solvePnP
  std::vector<cv::Point3f> object_points_;
  std::vector<cv::Point2f> image_points_;
  cv::Mat cam_mat_;
  cv::Mat dist_coef_;
  cv::Mat rvec_, tvec_;

  // variables for resutls
  cv::Mat R_cvmat_;
  bool Refreshed_, patternfound_;

  ros::AsyncSpinner spinner;

  // Subscribe and Publish topics
  ros::Publisher pos_pub_;
  ros::Subscriber locs_sub_;

  // variables for subscribing to image points and publishing world points
  std_msgs::Float64MultiArray locs_ordered_, pos_world_;

  std::thread findNRTThread, publishThread_;

public:
  FindNRT()
  : window_name_("checkerboard_img"), patternfound_(false),
  spinner(2)
  {
    // initializing subscriber and publisher
    pos_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/pos/world", 10);
  }

  ~FindNRT()
  {
    cv::destroyAllWindows();
  }

  void run()
  {
    if(spinner.canStart())
    {
      spinner.start();
      begin();
      end();
    }
  }

private:
  void begin()
  {
    populateParams();
    initCVParams();
    locs_sub_ = nh_.subscribe("/locs/ordered", 1, &FindNRT::callback, this);
    findNRTThread = std::thread(&FindNRT::find_nrt, this);
    publishThread_ = std::thread(&FindNRT::pub_recon_data, this);
  }

  void end(){
    spinner.stop();
  }

  void initCVParams()
  {
    // initializing video object
    cap_ = VideoCapture(cam_num);
    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name_, img_width_, img_hight_);
//initializing object points
    object_points_ = std::vector<cv::Point3f> (row_corners_*col_corners_,cv::Point3f());
    // initializing camera matrix
    cam_mat_ = cv::Mat(cam_mat_row_,cam_mat_col_,cv::DataType<double>::type);
    // initializing distortion coefficients
    dist_coef_ = cv::Mat(dist_coef_row_,dist_coef_col_,cv::DataType<double>::type);
    // initializing rotation and translation vectors
    rvec_ = cv::Mat(1,3,cv::DataType<double>::type);//CV_32F
    tvec_ = cv::Mat(1,3,cv::DataType<double>::type);
    // initializing Rotation matrix
    R_cvmat_ = cv::Mat(3,3,cv::DataType<double>::type);
  }

  template<typename T>
  void get_params(std::string param, T value) const
  {
    ros::param::get(param, value);
  }

  void populateParams() const{
    get_params("/findNRT/cam_num", cam_num);
    get_params("/findNRT/cam_params/image_width", img_width_);
    get_params("/findNRT/cam_params/image_hight", img_hight_);

    get_params("/findNRT/board_params/size/row_corners",row_corners_);
    get_params("/findNRT/board_params/size/col_corners",col_corners_);
    get_params("/findNRT/board_params/sqr_size",sqr_size_);

    get_params("/findNRT/cam_params/camera_matrix/rows", cam_mat_row_);
    get_params("/findNRT/cam_params/camera_matrix/cols", cam_mat_col_);

    get_params("/findNRT/cam_params/distortion_coefficients/rows", dist_coef_row_);
    get_params("/findNRT/cam_params/distortion_coefficients/cols", dist_coef_col_);
  }

  void find_nrt() // finds rotation matrix and translation vector using checkerboard
  {
    get_img();
    get_cam_param();

    cv::Size patternsize (row_corners_, col_corners_);
    patternfound_ = cv::findChessboardCorners(frame_,
                                             patternsize,
                                             image_points_,
                                             cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK );
    while (!patternfound_) // if pattern not detected, repeat process with new image
    {
      std::cout<<"No pattern found. Modify board postition, or camera postion/parameters"<<endl;
      get_img();
      show_imag();
      patternfound_ = cv::findChessboardCorners(frame_,
                                               patternsize,
                                               image_points_,
                                               cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK );
    }
    // checkerboard pattern should be defined now
    cv::drawChessboardCorners(frame_, patternsize, image_points_, patternfound_);
    show_imag();
    // at this point we have data in image_points

    // define object_points with data corresponding to the corners of the board:
    // the data points are defined based on the image points generated. y decreases
    // (going from bottom to top of image), then x increases (going from left
    // to right of image) to define the data points.
    for (int i = 0; i < col_corners_; ++i)
    {
      for (int j = 0; j < row_corners_; ++j)
      {
        object_points_.at( row_corners_*i+j ) = cv::Point3f((i+1)*sqr_size_, row_corners_*sqr_size_ - (j+1)*sqr_size_, 0);
      }
    }

    bool solved;
    bool use_extrinsic_guess;
    solved = cv::solvePnP (object_points_, image_points_,
                           cam_mat_, dist_coef_,
                           rvec_, tvec_);

    cv::Rodrigues(rvec_, R_cvmat_);

    stop_cap();
  }

  void pub_recon_data() // finds x,y,z world position of detected objects in the image
  {
    if(patternfound_)
    {
      int num_rob = locs_ordered_.data.size()/2;

      MatrixXd imgPts;
      imgPts.resize(3, num_rob);

      for (int i = 0; i < num_rob; ++i)
      {
        imgPts(0,i) = locs_ordered_.data[2*i];
        imgPts(1,i) = locs_ordered_.data[2*i+1];
        imgPts(2,i) = 1;
      }

      MatrixXd camMat; camMat.resize(3,3);
      for (int i = 0; i < 3; ++i)
      {
        for (int j = 0; j < 3; ++j)
        {
          camMat(i,j) = cam_mat_.at<double>(i,j);
        }
      }

      MatrixXd invCamMat; invCamMat.resize(3,3);
      invCamMat = camMat.inverse();

      MatrixXd p; p.resize(3,num_rob);
      p = invCamMat * imgPts;

      Vector3d nc;
      Vector3d temp001;
      temp001 << 0,0,1;
      MatrixXd R; R.resize(3,3);
      for (int i = 0; i < 3; ++i)
      {
        for (int j = 0; j < 3; ++j)
        {
          R(i,j) = R_cvmat_.at<double>(i,j);
        }
      }
      nc = R * temp001;

      double d = 0;

      Vector3d T;
      for (int i = 0; i < 3; ++i)
      {
        T(i) = tvec_.at<double>(i);
      }
      d = nc.dot(T) ;

      MatrixXd nxp; nxp.resize(1,num_rob);
      nxp = nc.transpose() * p;

      MatrixXd P; P.resize(3, num_rob);
      for (int i = 0; i < num_rob; ++i)
      {
        for (int j = 0; j < 3; ++j)
        {
          P(j,i) = d * p(j,i) / nxp(i);
        }
      }

      MatrixXd Pw; Pw.resize(3,num_rob);
      MatrixXd rtp; rtp.resize(3,num_rob);
      MatrixXd rtt; rtt.resize(3,1);
      rtp = R.transpose() * P;
      rtt = - R.transpose() * T;

      for (int i = 0; i < num_rob; ++i)
      {
        for (int j = 0; j < 3; ++j)
        {
          Pw(j,i) = rtp(j,i) + rtt(j);
        }
      }

      double pos[2][num_rob];
      for (int i = 0; i < num_rob; ++i)
      {
        pos[0][i] = + Pw(0,i);
        pos[1][i] = - Pw(1,i); // flip y to get the data represented in the conventional x-y-z frame with z pointing up (necessary for form ctrl)
        this->pos_world_.data.push_back(pos[0][i]);
        this->pos_world_.data.push_back(pos[1][i]);
      }

      pos_pub_.publish(this->pos_world_);

      pos_world_.data.clear();
    }
  }

  void get_img() // gets one image from camera
  {
    if (!this->cap_.isOpened())
    {
      return;
    }
    cv::Mat raw_img;
    this->cap_ >> raw_img;
    cv::cvtColor(raw_img, this->frame_, CV_BGR2GRAY);
    return;
  }

  void show_imag() // display image
  {
    if(this->frame_.empty())
    {
      return;
    }
    cv::imshow(window_name_, this->frame_);
    int key = cv::waitKey(1);
    return;
  }

  void stop_cap() //destroys video object and closes windows if destroyAllWindows is uncommented
  {
    cv::destroyAllWindows();
    this->cap_.release();
    return;
  }

  void get_cam_param() // gets camera parameters from yaml file
  {
    std::vector<double> a;
    std::vector<double> d;
    get_params("/findNRT/cam_params/camera_matrix/data", a);
    get_params("/findNRT/cam_params/distortion_coefficients/data", d);
    for (int i = 0; i < this->cam_mat_row_; ++i)
    {
      for (int j = 0; j < this->cam_mat_col_ ; ++j)
      {
        this->cam_mat_.at<double>(i,j) = a[cam_mat_col_*i + j];
      }
    }
    for (int i = 0; i < dist_coef_row_; ++i)
    {
      for (int j = 0; j < dist_coef_col_; ++j)
      {
        this->dist_coef_.at<double>(i,j) = d[cam_mat_col_*i + j];
      }
    }
    return;
  }

  void callback(const std_msgs::Float64MultiArray& locs) // callback to get ordered locations
  {
    this->locs_ordered_ = locs;
    pub_recon_data();
    return;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "findNRT");
  ros::NodeHandle nh;


  FindNRT fnrt;
  fnrt.run();

  if(!ros::ok())
  {
    return 0;
  }

  ros::shutdown();

  return EXIT_SUCCESS;
}
