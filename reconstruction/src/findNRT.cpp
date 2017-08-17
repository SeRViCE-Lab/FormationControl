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
  string window_name_;
  VideoCapture cap_;
  Mat frame_, cam_mat_, dist_coef_, rvec_, tvec_;

  // variables for board parameters
  double sqr_size_;
  int row_corners_, col_corners_, cam_mat_row_,
      cam_mat_col_, dist_coef_row_, dist_coef_col_;

  // variables for solvePnP
  vector<Point3f> object_points_;
  vector<Point2f> image_points_;

  // variables for resutls
  Mat R_cvmat_;
  bool Refreshed_, patternfound_, updateParams_;

  ros::AsyncSpinner spinner;

  // Subscribe and Publish topics
  ros::Publisher pos_pub_;
  ros::Subscriber locs_sub_;

  // variables for subscribing to image points and publishing world points
  std_msgs::Float64MultiArray locs_ordered_, pos_world_;

  thread findNRTThread, publishThread_;
public:
  FindNRT()
  : window_name_("checkerboard_img"), patternfound_(false),
  updateParams_(true),  spinner(2)
  {
    // initializing subscriber and publisher
    pos_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/pos/world", 10);
  }

  ~FindNRT()
  {
    destroyAllWindows();
  }

  void run()
  {
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


    populateParams();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    ROS_INFO("row_corners_ %d, col_corners_ %d, cam_mat_row_ %d, cam_mat_col_ %d",
        row_corners_, col_corners_, cam_mat_row_, cam_mat_col_);

    initCVParams();
    locs_sub_ = nh_.subscribe("/locs/ordered", 1, &FindNRT::callback, this);

    ROS_INFO(" :calling findnrt thread");
    findNRTThread = thread(&FindNRT::find_nrt, this);
    publishThread_ = thread(&FindNRT::pub_recon_data, this);
  }

  void end(){
    spinner.stop();
  }

  void initCVParams()
  {
    // initializing video object
    this->cap_ = VideoCapture(cam_num);
    namedWindow(window_name_, WINDOW_NORMAL);
    resizeWindow(window_name_, img_width_, img_hight_);
    //initializing object points
    object_points_ = vector<Point3f> (row_corners_*col_corners_,Point3f());
    // initializing camera matrix
    cam_mat_ = Mat(cam_mat_row_,cam_mat_col_,DataType<double>::type);
    // initializing distortion coefficients
    dist_coef_ = Mat(dist_coef_row_,dist_coef_col_,DataType<double>::type);
    // initializing rotation and translation vectors
    rvec_ = Mat(1,3,DataType<double>::type);//CV_32F
    tvec_ = Mat(1,3,DataType<double>::type);
    // initializing Rotation matrix
    R_cvmat_ = Mat(3,3,DataType<double>::type);

    std::cout<< " cam_mat_: " << cam_mat_ << std::endl;
  }

  template<typename T>
  void get_params(string&& param, T&& value)
  {
    nh_.getParam(param, value);
  }

  void populateParams() {
    get_params("/findNRT/cam_num", move(cam_num));
    get_params("/findNRT/cam_params/image_width", move(img_width_));
    get_params("/findNRT/cam_params/image_hight", move(img_hight_));

    get_params("/findNRT/board_params/size/row_corners", move(row_corners_));
    get_params("/findNRT/board_params/size/col_corners", move(col_corners_));
    get_params("/findNRT/board_params/sqr_size",move(sqr_size_));

    get_params("/findNRT/cam_params/camera_matrix/rows", move(cam_mat_row_));
    get_params("/findNRT/cam_params/camera_matrix/cols", move(cam_mat_col_));

    get_params("/findNRT/cam_params/distortion_coefficients/rows", move(dist_coef_row_));
    get_params("/findNRT/cam_params/distortion_coefficients/cols", move(dist_coef_col_));
    // updateParams_ = false;
  }

  void find_nrt() // finds rotation matrix and translation vector using checkerboard
  {
    ROS_INFO(" :within findnrt thread");
    get_img();
    ROS_INFO(":got after get image");
    get_cam_param();

    Size patternsize (row_corners_, col_corners_);
    patternfound_ = findChessboardCorners(frame_,
                                             patternsize,
                                             image_points_,
                                             CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK );
    while (!patternfound_) // if pattern not detected, repeat process with new image
    {
      cout<<"No pattern found. Modify board postition, or camera postion/parameters"<<endl;
      get_img();
      show_imag();
      patternfound_ = findChessboardCorners(frame_,
                                               patternsize,
                                               image_points_,
                                               CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK );
    }
    // checkerboard pattern should be defined now
    drawChessboardCorners(frame_, patternsize, image_points_, patternfound_);
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
        object_points_.at( row_corners_*i+j ) = Point3f((i+1)*sqr_size_, row_corners_*sqr_size_ - (j+1)*sqr_size_, 0);
      }
    }

    bool solved;
    bool use_extrinsic_guess;
    solved = solvePnP (object_points_, image_points_,
                           cam_mat_, dist_coef_,
                           rvec_, tvec_);

    Rodrigues(rvec_, R_cvmat_);

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
    ROS_INFO("within get_img");
    if (!this->cap_.isOpened())
    {
      ROS_FATAL("Can not open the camera. Ensure the driver is working ");
    }
    Mat raw_img;

    std::cout<< "populating raw img, cam_num_ is " << cam_num << "\n";

    this->cap_ >> raw_img;
    ROS_INFO("populated raw img");
    cv::cvtColor(raw_img, this->frame_, CV_BGR2GRAY);
    ROS_INFO("populated raw img");
  }

  void show_imag() // display image
  {
    if(this->frame_.empty())
    {
      return;
    }
    cv::imshow(window_name_, this->frame_);
    int key = waitKey(1);
    return;
  }

  void stop_cap() //destroys video object and closes windows if destroyAllWindows is uncommented
  {
    destroyAllWindows();
    this->cap_.release();
    return;
  }

  void get_cam_param() // gets camera parameters from yaml file
  {
    vector<double> a;
    vector<double> d;
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
    std::cout << "locs: " << "here in callback" << std::endl;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "findNRT");

  FindNRT fnrt;
  fnrt.run();

  if(!ros::ok())
  {
    return 0;
  }

  ros::shutdown();

  return EXIT_SUCCESS;
}
