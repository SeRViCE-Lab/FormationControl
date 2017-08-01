#include <vector>
#include <string>
#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
// #include <ros/spinner.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <std_msgs/ColorRGBA.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

using namespace cv;
using namespace std;

class FindNRT
{
private:
  // node handle, image, and video variables
  ros::NodeHandle nh_;
  int cam_num, img_width, img_hight;
  std::string windowName;
  VideoCapture cap;
  cv::Mat frame;

  // variables for board parameters
  double sqr_size;
  int row_corners, col_corners;

  // variables for camera parameters
  int cam_mat_row, cam_mat_col;
  int dist_coef_row, dist_coef_col;

  // variables for solvePnP
  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> image_points;
  cv::Mat cam_mat;
  cv::Mat dist_coef;
  cv::Mat rvec, tvec;

  // variables for resutls
  cv::Mat R_cvmat;
  bool Refreshed;

  // Subscribe and Publish topics
  ros::Publisher pos_pub_;
  ros::Subscriber locs_sub_;

  // variables for subscribing to image points and publishing world points
  std_msgs::Float64MultiArray locs_ordered, pos_world;



public:
  FindNRT()
  {
    // initializing video object
    nh_.getParam("/findNRT/cam_num", cam_num);
    nh_.getParam("/findNRT/cam_params/image_width", img_width);
    nh_.getParam("/findNRT/cam_params/image_hight", img_hight);
    windowName = "checkerboard_img";
    cap = VideoCapture(cam_num);
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, img_width, img_hight);

    // initializing board parameters
    nh_.getParam("/findNRT/board_params/size/row_corners",row_corners);
    nh_.getParam("/findNRT/board_params/size/col_corners",col_corners);
    nh_.getParam("/findNRT/board_params/sqr_size",sqr_size);

    //initializing object points
    object_points = std::vector<cv::Point3f> (row_corners*col_corners,cv::Point3f());
    // initializing camera matrix
    nh_.getParam("/findNRT/cam_params/camera_matrix/rows", cam_mat_row);
    nh_.getParam("/findNRT/cam_params/camera_matrix/cols", cam_mat_col);
    cam_mat = cv::Mat(cam_mat_row,cam_mat_col,cv::DataType<double>::type);
    // initializing distortion coefficients
    nh_.getParam("/findNRT/cam_params/distortion_coefficients/rows", dist_coef_row);
    nh_.getParam("/findNRT/cam_params/distortion_coefficients/cols", dist_coef_col);
    dist_coef = cv::Mat(dist_coef_row,dist_coef_col,cv::DataType<double>::type);
    // initializing rotation and translation vectors
    rvec = cv::Mat(1,3,cv::DataType<double>::type);//CV_32F
    tvec = cv::Mat(1,3,cv::DataType<double>::type);
    // initializing Rotation matrix
    R_cvmat = cv::Mat(3,3,cv::DataType<double>::type);

    // initializing subscriber and publisher
    locs_sub_ = nh_.subscribe("/locs/ordered", 1, &FindNRT::callback, this);
    pos_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/pos/world", 10);

    // initializing locs_ordered and pos_world
    locs_ordered.data.clear();
    pos_world.data.clear();

  }

  ~FindNRT()
  {
    cv::destroyAllWindows();
  }

  void find_nrt()
  {
    get_img();
    get_cam_param();

    bool patternfound = false;
    cv::Size patternsize (row_corners, col_corners);
    patternfound = cv::findChessboardCorners(frame,
                                             patternsize,
                                             image_points,
                                             cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK );
    while (!patternfound) // if pattern not detected, repeat process with new image
    {
      std::cout<<"No pattern found. Modify board postition, or camera postion/parameters"<<endl;
      get_img();
      show_imag();
      patternfound = cv::findChessboardCorners(frame,
                                               patternsize,
                                               image_points,
                                               cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK );
    }
    cv::drawChessboardCorners(frame, patternsize, image_points, patternfound);
    show_imag();
    // at this point we should have data in image_points

    // define object_points with data corresponding to the corners of the board
    for (int i = 0; i < col_corners; ++i)
    {
      for (int j = 0; j < row_corners; ++j)
      {
        object_points.at( row_corners*i+j ) = cv::Point3f((i+1)*sqr_size, row_corners*sqr_size - (j+1)*sqr_size, 0);
      }
    }

    bool solved;
    bool use_extrinsic_guess;
    solved = cv::solvePnP (object_points, image_points,
                           cam_mat, dist_coef,
                           rvec, tvec);

    cv::Rodrigues(rvec, R_cvmat);

    //// uncomment lines below to print out R and T to terminal window
    // cout<<"R_cvmat = " <<endl << R_cvmat << endl;
    // cout<<"tvec = " <<endl << tvec << endl;
    stop_cap();
  }

  void pub_recon_data()
  {
    if (locs_ordered.data.empty())
    {
      return;
    }

    int num_rob = locs_ordered.data.size()/2;

    double img_pts[3][num_rob];
    for (int i = 0; i < num_rob; ++i)
    {
      img_pts[0][i] = locs_ordered.data[2*i];
      img_pts[1][i] = locs_ordered.data[2*i+1];
      img_pts[2][i] = 1;
    }
    double inv_cam_mat[3][3];
    inv_cam_mat[0][0] = 1/cam_mat.at<double>(0,0);
    inv_cam_mat[0][1] = 0;
    inv_cam_mat[0][2] = -cam_mat.at<double>(0,2)/cam_mat.at<double>(0,0);
    inv_cam_mat[1][0] = 0;
    inv_cam_mat[1][1] = 1/cam_mat.at<double>(1,1);
    inv_cam_mat[1][2] = -cam_mat.at<double>(1,2)/cam_mat.at<double>(1,1);
    inv_cam_mat[2][0] = 0;
    inv_cam_mat[2][1] = 0;
    inv_cam_mat[2][2] = 1;

    double p [3][num_rob];
    for (int i = 0; i < num_rob; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        p[j][i] = inv_cam_mat[j][0] * img_pts[0][i] +
                  inv_cam_mat[j][1] * img_pts[1][i] +
                  inv_cam_mat[j][2] * img_pts[2][i] ;
      }
    }

    double nc[3];
    for (int i = 0; i < 3; ++i)
    {
      nc[i]= this->R_cvmat.at<double>(3,i);
    }

    double d = 0;
    for (int i = 0; i < 3; ++i)
    {
      d += nc[i] * this->tvec.at<double>(i);
    }

    double nxp[num_rob];
    for (int i = 0; i < num_rob; ++i)
    {
      nxp[i] = nc[0] * p[0][i] +
               nc[1] * p[1][i] +
               nc[2] * p[2][i] ;
    }

    double cap_p[3][num_rob];
    for (int i = 0; i < num_rob; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        cap_p[j][i] = d * p[j][i] / nxp[i];
      }
    }

    double pw [3][num_rob];
    double rt [3][3];
    double rtp [3][num_rob];
    double rtt [3];
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        rt[i][j] = this->R_cvmat.at<double>(j,i);
      }
    }
    for (int i = 0; i < num_rob; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        rtp[j][i] = rt[j][0] * p[0][i] +
                    rt[j][1] * p[1][i] +
                    rt[j][2] * p[2][i] ;
      }
    }
    for (int i = 0; i < 3; ++i)
    {
      rtt[i] = 0;
      for (int j = 0; j < 3; ++j)
      {
        rtt[i] += rt[i][j]*this->tvec.at<double>(j);
      }
    }
    for (int i = 0; i < num_rob; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        pw[j][i] = rtp[j][i] - rtt[j];
      }
    }

    double pos[2][num_rob];
    for (int i = 0; i < num_rob; ++i)
    {
      pos[0][i] = - pw[0][i];
      pos[1][i] = + pw[1][i];
      this->pos_world.data.push_back(pos[0][i]);
      this->pos_world.data.push_back(pos[1][i]);
    }

    pos_pub_.publish(this->pos_world);

    pos_world.data.clear();

  }

private:

  void get_img() // gets one image from camera
  {
    if (!this->cap.isOpened())
    {
      return;
    }
    cv::Mat raw_img;
    this->cap >> raw_img;
    cv::cvtColor(raw_img, this->frame, CV_BGR2GRAY);
    return;
  }

  void show_imag() // display image
  {
    if(this->frame.empty())
    {
      return;
    }
    cv::imshow(windowName, this->frame);
    int key = cv::waitKey(1);
    return;
  }

  void stop_cap() //close all windows and destroys video object
  {
    cv::destroyAllWindows();
    this->cap.release();
    return;
  }

  void get_cam_param() // gets camera parameters from yaml file
  {
    std::vector<double> a;
    std::vector<double> d;
    this->nh_.getParam("/findNRT/cam_params/camera_matrix/data",a);
    this->nh_.getParam("/findNRT/cam_params/distortion_coefficients/data",d);
    for (int i = 0; i < this->cam_mat_row; ++i)
    {
      for (int j = 0; j < this->cam_mat_col ; ++j)
      {
        this->cam_mat.at<double>(i,j) = a[cam_mat_col*i + j];
      }
    }
    for (int i = 0; i < dist_coef_row; ++i)
    {
      for (int j = 0; j < dist_coef_col; ++j)
      {
        this->dist_coef.at<double>(i,j) = d[cam_mat_col*i + j];
      }
    }
    return;
  }

  void callback(const std_msgs::Float64MultiArray& locs) // callback for ordered location subscriber
  {
    this->locs_ordered = locs;
    pub_recon_data();
    return;
  }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "findNRT");
  ros::NodeHandle nh;

  ros::Rate r(30);

  FindNRT f;
  f.find_nrt();
  f.pub_recon_data();
  ros::spin();
  r.sleep();

  if(!ros::ok())
  {
    return 0;
  }

  ros::shutdown();

}
