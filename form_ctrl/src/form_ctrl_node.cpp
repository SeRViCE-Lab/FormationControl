// #include <ros/spinner.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/Image.h>
// #include <image_transport/image_transport.h>
// #include <std_msgs/ColorRGBA.h>
#include <vector>
#include <string>
#include <iostream>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <ctime>
// #include <cstdlib>
#include <sys/time.h>
#include <fstream> // to be removed later on once testing is complete
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32.h"


using namespace cv;
using namespace std;
using namespace Eigen;

// global variables
int itr = 1;
bool first_time = true;
double L [6][6]; // 3 rob;
MatrixXd lnr_mat(6,6); // 3 rob
// double L [8][8]; // 4 rob;
// MatrixXd lnr_mat(8,8); //4 rob

int num_rows;
bool use_adaptive_vel_ctrl = false;
bool use_PID_ctrl = true;
int idx = -1;

// Size s = Size(640,480);
// VideoWriter outputVideo("/home/hivemind/sandbox/vid.mjpg",('M','J','P','G'), 10, s, true  );


class Ctrl
{
private:
// variables for robot parameters
int num_rob_;
std::vector<std::string> rob_names_,
                         rob_ctrl_topic_names_,
                         rob_ang_vel_topic_names_;

// node handle and Subscribe and Publish topics
ros::NodeHandle nh_;
ros::Subscriber pos_world_sub_,
                pos_image_sub_;
std::vector<ros::Publisher> ctrl_cmd_pub_array_,
                            ang_vel_pub_array_;

// variables used for formation control and PID controller
std::vector<double> ctrl_, vd_,
                    thte_, thtc_, thtc_old_, thtr_, tht0_old_, tht0_new_,
                    a_hat_old_, a_hat_new_, b_hat_old_, b_hat_new_,
                    error_sum_,
                    vm_new_, vm_old_,
                    vel_, ang_;
double t_old_, t_new_,
       v_est_;
int mem_, mem_filled_;


// variables for robot position
std_msgs::Float64MultiArray pos_world_new_,
                            pos_image_new_;

std::vector<MatrixXd> pos_world_history_,
                      pos_image_history_;

// GAINS and thresholds:
double vd_gain_, v_gain_, vmax_,
       km_, ka_, kh_, kp_, ki_,
       a_bound_, b_bound_,
       gain_, speed_gain_,
       delay_;
int thresh_pix_;
bool update;

std::string window_name = "heading command";
std::string window_name2 = "reconstruction";

// file stream. To be removed for final version
ofstream outfile;

public:
Ctrl()   // class constructor
{
        nh_.getParam("/form_ctrl_node/Rob_Params/num_rob", num_rob_); // number of robots
        for (int num_rob = 0; num_rob < num_rob_; ++num_rob) // loop through robots to generate topic names
        {
                std::string rob_id = "sph" + std::to_string(num_rob);
                std::string rob_name;
                nh_.getParam("/form_ctrl_node/Rob_Params/"+rob_id, rob_name);
                rob_names_.push_back(rob_name);
                rob_ctrl_topic_names_.push_back("/" + rob_names_[num_rob] + "/cmd_vel");
                rob_ang_vel_topic_names_.push_back("/" + rob_names_[num_rob] + "/set_angular_velocity");
                ctrl_cmd_pub_array_.push_back(nh_.advertise<geometry_msgs::Twist>(rob_ctrl_topic_names_[num_rob], 1));
                ang_vel_pub_array_.push_back(nh_.advertise<std_msgs::Float32>(rob_ang_vel_topic_names_[num_rob], 1));
        }
        pos_image_sub_ = nh_.subscribe("/locs/ordered", 1, &Ctrl::pos_image_callback, this);
        pos_world_sub_ = nh_.subscribe("/pos/world", 1, &Ctrl::pos_world_callback, this);

        // initializing variables used for formation control and PID controller
        ctrl_.resize(2*num_rob_);
        vd_.resize(num_rob_);
        t_old_ = 0;
        t_new_ = 0;
        thte_.resize(num_rob_);
        thtc_.resize(num_rob_);
        thtc_old_.resize(num_rob_);
        thtr_.resize(num_rob_);
        tht0_old_.resize(num_rob_);
        tht0_new_.resize(num_rob_);
        v_est_ = 0;
        mem_ = 0; //5
        mem_filled_ = 0;
        a_hat_old_.resize(num_rob_);
        a_hat_new_.resize(num_rob_);
        b_hat_old_.resize(num_rob_);
        b_hat_new_.resize(num_rob_);
        vm_new_.resize(num_rob_);
        vm_old_.resize(num_rob_);
        vel_.resize(num_rob_);
        ang_.resize(num_rob_);
        for (int i = 0; i < num_rob_; ++i)
        {
                ang_[i] = 0;
        }
        error_sum_.resize(num_rob_);

        // initialize robot position variables
        pos_world_new_.data.clear();
        pos_image_new_.data.clear();

        pos_world_history_.resize(num_rob_);
        pos_image_history_.resize(num_rob_);
        for (int i = 0; i < num_rob_; ++i)
        {
                pos_world_history_[i]=MatrixXd(3,mem_+1);
                pos_image_history_[i]=MatrixXd(3,mem_+1);
        }

        // initializing GAINS and thresholds
        nh_.getParam("/form_ctrl_node/Gains_Thresholds/vd_gain_", vd_gain_);
        nh_.getParam("/form_ctrl_node/Gains_Thresholds/v_gain_", v_gain_);
        nh_.getParam("/form_ctrl_node/Gains_Thresholds/kh_", kh_);
        nh_.getParam("/form_ctrl_node/Gains_Thresholds/thresh_pix_", thresh_pix_);
        nh_.getParam("/form_ctrl_node/Gains_Thresholds/vmax_", vmax_);
        nh_.getParam("/form_ctrl_node/Gains_Thresholds/km_", km_);
        nh_.getParam("/form_ctrl_node/Gains_Thresholds/ka_", ka_);
        nh_.getParam("/form_ctrl_node/Gains_Thresholds/a_bound_", a_bound_);
        nh_.getParam("/form_ctrl_node/Gains_Thresholds/b_bound_", b_bound_);
        nh_.getParam("/form_ctrl_node/Gains_Thresholds/gain_", gain_);
        nh_.getParam("/form_ctrl_node/Gains_Thresholds/speed_gain_", speed_gain_);
        nh_.getParam("/form_ctrl_node/Gains_Thresholds/kp_", kp_);
        nh_.getParam("/form_ctrl_node/Gains_Thresholds/ki_", ki_);
        nh_.getParam("/form_ctrl_node/Delay", delay_);

        cv::namedWindow(window_name, cv::WINDOW_NORMAL);
        cv::resizeWindow(window_name, 640, 480);
        cv::namedWindow(window_name2, cv::WINDOW_NORMAL);
        cv::resizeWindow(window_name2, 400, 400);

        outfile.open("/home/hivemind/sandbox/pub_data.txt");


}

~Ctrl() {   // class destructor
        // file stream. To be removed for final version
        outfile.close();
}

std_msgs::Float64MultiArray get_pos_world_new() // returns world position fetched
{
        return pos_world_new_;
}

std_msgs::Float64MultiArray get_pos_image_new() // returns image postition fetched
{
        return pos_image_new_;
}

void form_ctrl()  // formation control
{
        int iitr = 0;
        for (int num_rob = 0; num_rob < num_rob_; ++num_rob)
        {
                iitr = (itr-1) * num_rob_ + (num_rob+1);
                if (iitr == 1) // initialize some variables in first iteration
                {
                        tht0_new_[num_rob] = 0;
                        tht0_old_[num_rob] = 0;
                        thtc_[num_rob] = 0;
                        thtc_old_[num_rob] = 0;
                }

                ros::spinOnce();

                t_new_ = static_cast<double>(clock())/CLOCKS_PER_SEC;

                if (iitr == 1) // initialize some variables in first iteration
                {
                        for (int i = 0; i < num_rob_; ++i)
                        {
                                double x_init_w, y_init_w;
                                x_init_w = pos_world_new_.data[2*i];
                                y_init_w = pos_world_new_.data[2*i+1];

                                double x_init_i, y_init_i;
                                x_init_i = pos_image_new_.data[2*i];
                                y_init_i = pos_image_new_.data[2*i+1];

                                for (int j = 0; j < mem_+1; ++j)
                                {

                                        pos_world_history_[i](0,j) = static_cast<double>(t_new_);
                                        pos_world_history_[i](1,j) = x_init_w;
                                        pos_world_history_[i](2,j) = y_init_w;

                                        pos_image_history_[i](0,j) = static_cast<double>(t_new_);
                                        pos_image_history_[i](1,j) = x_init_i;
                                        pos_image_history_[i](2,j) = y_init_i;
                                }
                        }
                }

                if (itr >= 2)
                {
                        estimate_heading(iitr, num_rob); // estimate the robot's heading (thte_)
                }

                form_ctrl_free_scale(); //apply free-scale formation control and find control commands (ctrl_)

                find_vd(); // find desired velocity (vd_) based on ctrl_

                double v [2];
                v[0] = ctrl_[2*num_rob];
                v[1] = ctrl_[2*num_rob + 1];

                if (idx != -1) // idx != -1 if sufficient motion exists
                {
                        // find control angle thtc_
                        thtc_[num_rob] = bound(atan2(v[1],v[0]) * 180 / M_PI); // angle in degrees
                }

                if (itr >= 2)
                {
                        // estimate tht0: initial heading offset of the robot
                        estimate_tht0(thtc_old_[num_rob], thte_[num_rob], num_rob); //TODO: write code of fxn
                }

                thtr_[num_rob] = thtc_[num_rob] - tht0_new_[num_rob]; // find robot velocity command angle in degrees
                thtr_[num_rob] = bound(thtr_[num_rob]);
                outfile << "##############Comparing thtc and thte#############" <<endl;
                outfile << "THTC = " << thtc_[num_rob] << endl;
                outfile << "THTE = " << thte_[num_rob] << endl;
                outfile << "Ideally, they should be equal" << endl;
                outfile << "##################################################" <<endl;

                double vel_temp; // Use PID controller
                if (itr == 1)
                {
                        vm_new_ [num_rob] = 0;
                        a_hat_new_ [num_rob] = 0;
                        b_hat_new_ [num_rob] = 0;
                }
                else // itr >= 2
                {
                        if (use_adaptive_vel_ctrl == true)
                        {
                                // apply_adaptive_vel_ctrl(num_rob, dt);

                        }
                        else
                        {
                                if (use_PID_ctrl == true)
                                        vel_temp = apply_pid_ctrl(num_rob);
                        }
                }

                // find speed command value
                double vel;
                if (use_adaptive_vel_ctrl == true) // adaptive_ctrl doesn't work yet
                        vel = a_hat_new_[num_rob]*v_est_ + b_hat_new_[num_rob]*vd_[num_rob];
                else
                {
                        if (use_PID_ctrl == true)
                                vel = vel_temp;
                }
                cout<< vel << endl; ////////////////////////////
                vel = vel;
                vel = min(vel,vmax_);
                vel = find_max(vel, 0.01);
                vel_[num_rob] = vel;
                if (idx != -1)
                {
                        ang_[num_rob] = thtr_[num_rob]; // update command velocity angle when sufficient motion exists
                }

                cout << "robot: " << num_rob << endl << " vel: " << vel_[num_rob] << "  ang: " << ang_[num_rob] << endl;//////////////////////////////////////////////////////////////////

                // find x and y velocity commands
                double vel_x = vel_[num_rob] * cos(ang_[num_rob]*M_PI/180);
                double vel_y = vel_[num_rob] * sin(ang_[num_rob]*M_PI/180);

                // issue commands;
                std_msgs::Float32 set_angular_velocity_command;
                set_angular_velocity_command.data = 255.0;

                geometry_msgs::Twist command;
                command.linear.x = static_cast<float>(vel_x);
                command.linear.y = static_cast<float>(vel_y);
                command.linear.z = 0;
                command.angular.x = 0;
                command.angular.y = 0;
                command.angular.z = 0;

                ang_vel_pub_array_ [num_rob].publish(set_angular_velocity_command);
                ctrl_cmd_pub_array_ [num_rob].publish(command);
                ros::Duration(delay_).sleep();

                outfile << "itr: " << itr << " iitr: " << iitr << "   rob num: " << num_rob;
                outfile << " estimated tht0: " << tht0_new_[num_rob];
                outfile << " command vel: " << vel_[num_rob] << " command heading: " << ang_[num_rob];
                outfile << endl << " pos world: " << endl;
                outfile << pos_world_new_;
                outfile << " x_vel: " << vel_x << " y_vel: " << vel_y << endl;
                for ( int i = 0; i < num_rob_; ++i)
                {
                        outfile << "pos_world_history[" << i <<"] = " << pos_world_history_[i] << endl;
                        outfile << "pos_image_history[" << i <<"] = " << pos_image_history_[i] << endl <<endl;
                }

                if (num_rob == (num_rob_-1))
                        outfile << endl << endl << endl << endl << endl;

                ros::spinOnce();

                // Display visual feedback
                Mat frame(480, 640, CV_8UC3, Scalar(0)); //image frame and thtc, thte, thtr vectors
                Mat frame_w(400, 400, CV_8UC3, Scalar(0)); // 3d reconstruction frame
                for ( int i = 0; i < num_rob_; ++i )
                {
                        Point center;
                        center.x = pos_image_new_.data[2*i];
                        center.y = pos_image_new_.data[2*i+1];
                        int radius = 5;
                        int thickness = 3;
                        circle(frame, center, radius, Scalar(0, 255, 255), thickness); // display robot in image frame

                        Point center_world, origin_world, x_dir_world, y_dir_world;
                        origin_world.x = 200 * (0 + 1);
                        origin_world.y = 200 * (-0 + 1);
                        x_dir_world.x = 200 * (0.5 + 1);
                        x_dir_world.y = 200 * (-0 + 1);
                        y_dir_world.x = 200 * (0 + 1);
                        y_dir_world.y = 200 * (-0.5 + 1);
                        center_world.x = 200 * (pos_world_new_.data[2*i] +1);
                        center_world.y = 200 * (-pos_world_new_.data[2*i+1] +1); // flip y to plot in frame with z pointing down.
                        circle(frame_w, center_world, radius, Scalar(0, 255, 255), thickness); // display robot in 3d reconstruction frame
                        line(frame_w, origin_world, x_dir_world, Scalar(255,0,0), thickness); // x-axis in 3d reconstruction
                        line(frame_w, origin_world, y_dir_world, Scalar(0,0,255), thickness); // y-axis in 3d reconstruction

                        if (i == num_rob)
                        {
                                Point end_pt;
                                end_pt.x = (center.x + vel_x);
                                end_pt.y = (center.y - vel_y);
                                line(frame, center, end_pt, Scalar(0, 255, 0), thickness); //  command velocity vector (green)
                        }

                        Point end_pt_heading;
                        end_pt_heading.x = (center.x + 20*cos(thte_[i]*M_PI/180));
                        end_pt_heading.y = (center.y - 20*sin(thte_[i]*M_PI/180));
                        line(frame, center, end_pt_heading, Scalar(255, 50, 0), thickness); // estimated heading (blue)

                        Point end_pt_tht0;
                        end_pt_tht0.x = (center.x + 20*cos(tht0_new_[i]*M_PI/180));
                        end_pt_tht0.y = (center.y - 20*sin(tht0_new_[i]*M_PI/180));
                        line(frame, center, end_pt_tht0, Scalar(0, 0, 255), thickness); // estimated tht0 (red)

                        Point txt_pt;
                        txt_pt.x = min(center.x + 5, 630);
                        txt_pt.y = max(center.y - 5, 0);
                        putText(frame, std::to_string(i+1), txt_pt, FONT_HERSHEY_PLAIN, 2, Scalar(255,255,255), 1); // robot tag number
                }
                // outputVideo << frame;
                imshow(window_name, frame);
                imshow(window_name2, frame_w);

                int key = waitKey(1);
                updata_history (num_rob);

                ros::spinOnce();

        }
        update_old_new_variables();
}

private:


void pos_world_callback(const std_msgs::Float64MultiArray& pos_world_new)
{
        this->pos_world_new_ = pos_world_new;
        return;
}

void pos_image_callback(const std_msgs::Float64MultiArray& pos_image_new)
{
        this->pos_image_new_ = pos_image_new;
        return;
}

void form_ctrl_free_scale()   // free scale formation control function
{
        if (first_time)
        {
                first_time = false;
                // x,y pairs for the shape
                // N-gon formation (circle)
                double sep_ang = 360 / this->num_rob_;
                double q_ang [this->num_rob_];
                for (int i = 0; i < this->num_rob_; ++i)
                {
                        q_ang[i] = sep_ang*i;
                }
                double pos_des [2][this->num_rob_];
                for (int i = 0; i < this->num_rob_; ++i)
                {
                        pos_des[0][i] = cos(q_ang[i] * M_PI / 180.0);
                        pos_des[1][i] = sin(q_ang[i] * M_PI / 180.0);
                }
                // adjacency matrix of all graphs with 4 agents
                int num_grpahs = 1;
                double adj [this->num_rob_][this->num_rob_][num_grpahs] = {0};
                for (int i = 0; i < this->num_rob_; ++i)
                {
                        for (int j = 0; j < this->num_rob_; ++j)
                        {
                                if (i==j)
                                {
                                        adj[i][j][0] = 0;
                                }
                                else
                                {
                                        adj[i][j][0] = 1;
                                }
                        }
                }

                // TODO: add rearrangment of robots to travel shorter distance.

                // TODO: find gains that stabilize all adjacency matrices, find laplacian matrix and normalize it

                // get real laplacian matrix data produced by Matlab, store it in L, and normalize it
                std::vector<double> lnr_3;
                // std::vector<double> lnr_3_temp;
                //
                // std::string temp_lnr_name,temp_lnr_name_base;
                // this->nh_.getParam("/form_ctrl_node/CVX_Optimization/rows", num_rows);
                // temp_lnr_name_base = "LnR_3_";
                // lnr_3.resize(num_rows,num_rows);
                // for (int i = 0; i < num_rows; ++i)
                // {
                //   temp_lnr_name = "/form_ctrl_node/CVX_Optimization/" + temp_lnr_name_base + to_string(i+1);
                //   this->nh_.getParam(temp_lnr_name, lnr_3_temp);
                //
                //   for(int j = 0; j < num_rows; ++j)
                //   {
                //     lnr_3[num_rows*i + j] = lnr_3_temp[j];
                //   }
                //
                // }
                //

                this->nh_.getParam("/form_ctrl_node/CVX_Optimization/rows", num_rows);
                this->nh_.getParam("/form_ctrl_node/CVX_Optimization/LnR_3", lnr_3);
                double max_lnr = 0;

                for (int i = 0; i < num_rows; ++i)
                {
                        for (int j = 0; j < num_rows; ++j)
                        {
                                lnr_mat(i,j) = lnr_3[num_rows*i+j];
                                max_lnr = find_max (max_lnr, abs(lnr_mat(i,j)));
                        }
                }
                if (max_lnr != 0)
                        lnr_mat / max_lnr;
                outfile << "LnR matrix : " << endl;
                outfile << lnr_mat << endl;

        }

        MatrixXd q;
        q.resize(2*this->num_rob_,1);
        double q_max = 0;
        for (int i = 0; i < 2*this->num_rob_; ++i)
        {
                q(i) = this->pos_world_new_.data[i];
                q_max = find_max(q_max, abs(q(i)));
        }
        if (q_max != 0)
                q /= q_max;

        MatrixXd dq;
        dq.resize(2 * this->num_rob_,1);
        dq = lnr_mat*q;

        for (int i = 0; i < 2*this->num_rob_; ++i)
        {
                this->ctrl_[i] = dq(i);
        }
}

void find_vd()   // find desired velocity
{

        bool scale = false;
        double vd_max = 0;
        for (int i = 0; i < this->num_rob_; ++i)
        {
                this->vd_[i] = this->vd_gain_ * find_norm(this->ctrl_[2*i], this->ctrl_[2*i+1]);
                if (this->vd_[i] > this->vmax_) // check if any vd value is larger than max velocity
                {
                        scale = true;
                        if (this->vd_[i]>vd_max)
                        {
                                vd_max = this->vd_[i];
                        }
                }
        }

        if (scale) // if there exists a vd value > max vel, scale all vd values
        {
                double scale_factor = this->vmax_ / vd_max;
                for (int i = 0; i < this->num_rob_; ++i)
                {
                        this->vd_[i] *= scale_factor;
                }
        }


}

void estimate_heading (int iitr, int num_rob)   // estimate heading of robot based on data from image
{
        MatrixXd pos_w; pos_w.resize(3, mem_+1); // with time
        MatrixXd pos_i; pos_i.resize(3, mem_+1); // with time
        pos_w = this->pos_world_history_[num_rob];
        pos_i = this->pos_image_history_[num_rob];

        MatrixXd d_pos_w; d_pos_w.resize(2, mem_+1); // difference in xy position
        MatrixXd d_pos_i; d_pos_i.resize(2, mem_+1); // difference in xy position

        outfile << "~~~~~~~~~~~~~~~~~~~~~~Heading estimation~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
        outfile << "Robot Number = " << num_rob << endl;
        outfile << "History location:" <<endl;
        outfile << "      WORLD: " <<endl << pos_w << endl;
        outfile << "      IMAGE: " <<endl << pos_i << endl;
        outfile << "New location:" <<endl;
        outfile << "      WORLD: " <<endl << pos_world_new_.data[2*num_rob] << ", " << pos_world_new_.data[2*num_rob+1]  << endl;
        outfile << "      IMAGE: " <<endl << pos_image_new_.data[2*num_rob] << ", " << pos_image_new_.data[2*num_rob+1]  << endl;

        for (int i = 0; i < mem_+1; ++i)
        {
                for (int j = 0; j < 2; ++j)
                {
                        d_pos_w(j,i) = pos_world_new_.data[2*num_rob+j] - pos_w(j+1,i);
                        d_pos_i(j,i) = pos_image_new_.data[2*num_rob+j] - pos_i(j+1,i);
                }
        }

        outfile << "Difference between new and history x and y: " << endl;
        outfile << "      WORLD: " <<endl << d_pos_w << endl;
        outfile << "      IMAGE: " <<endl << d_pos_i << endl;

        double dist = find_norm( d_pos_w(0, mem_), d_pos_w(1, mem_) );

        outfile << "Distance: " << endl;

        double dist_pix [mem_ + 1];
        for (int i = 0; i < mem_ + 1; ++i)
        {
                dist_pix[i] = find_norm(d_pos_i(0,i), d_pos_i(1,i));
        }

        outfile << "      WORLD: " <<endl << dist << endl;
        outfile << "      IMAGE: " <<endl << dist_pix[0] << endl;

        idx = -1;
        for (int i = 0; i < mem_ + 1; ++i)
        {
                if (dist_pix[i] > this->thresh_pix_)
                {
                        idx = i;
                }
        }

        if (idx != -1)
        {
                double hd [2];
                hd[0] = d_pos_w(0,idx);
                hd[1] = d_pos_w(1,idx);
                this->thte_[num_rob] = atan2(hd[1],hd[0]) * 180 / M_PI; // angle in degrees
                this->thte_[num_rob] = bound(this->thte_[num_rob]);
                // this->thte_[num_rob] = (this->thte_[num_rob]);

                double dt;
                dt = this->t_new_ - this->pos_world_history_[num_rob](0,idx);

                this->v_est_ = this->v_gain_ * dist / dt;
                outfile << " DISTANCE = " << dist <<endl;
                outfile << " DT = " << dt <<endl;
                outfile << " v_est_ = " << this->v_est_ << endl;
        }
        else
        {
                this->v_est_ = 0;
                outfile << " v_est_ = " << this->v_est_ << endl;

        }

        outfile << "Is pixel distance greater than threshold? :: " << endl;
        bool isit = dist_pix[0] > this->thresh_pix_;
        outfile << isit << endl;

        // cout << "vest  = " << v_est_ <<endl;
        outfile << "Estimated heading: " <<endl << this->thte_[num_rob] << endl;
        outfile << "~~~~~~~~~~~~~~~~~~~~~~Heading estimation DONE~~~~~~~~~~~~~~~~~~~~~~~" << endl;


}

void estimate_tht0(double thtc, double thte, int num_rob) // estimate tht0, the initial heading of the robot
{
        // cout<<"Estimating tht 0"<<endl; /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        outfile << "````````````````````````````````THT0 estimation````````````````````````````````" << endl;

        if (isnan(thte) || idx == -1)
        {
                this->tht0_new_[num_rob] = this->tht0_old_[num_rob];
                outfile << "keeping old value: " << tht0_new_[num_rob] << endl;
        }
        else
        {
                double tht_diff = ang_diff(thtc, thte); // thte-thtc angle in degrees

                this->tht0_new_[num_rob] = this->kh_ * (tht_diff - this->tht0_old_[num_rob]) + this->tht0_old_[num_rob];

                outfile << "calculating new value: " << endl;
                outfile << "thte = " << thte << endl;
                outfile << "thtc = " << thtc << endl;
                outfile << "tht_diff = " << tht_diff << endl;
                outfile << "kh_ * tht_diff = " << this->kh_ * tht_diff << endl;
                outfile << "tht0 old = " << this->tht0_old_[num_rob] << endl;
                outfile << "new value: " << tht0_new_[num_rob] << endl;

        }
        outfile << "````````````````````````````````THT0 estimation DONE```````````````````````" << endl;
}

void apply_adaptive_vel_ctrl(int num_rob, double dt) // adaptive velocity control
{

        // from v 8.2 (Matlab)
        // this->vm_new_[num_rob] = dt * (-this->km_*(this->vm_old_[num_rob] - this->vm_new_[num_rob])) + this->vm_old_[num_rob];
        // double ev = this->v_est_ - this->vm_new_[num_rob];
        // this->a_hat_new_[num_rob] = dt * (-this->ka_ * ev * this->v_est_) + this->a_hat_old_[num_rob];
        // this->a_hat_new_[num_rob] = find_sign(this->a_hat_new_[num_rob]) * min(abs(this->a_hat_new_[num_rob]), this->a_bound_);
        // this->b_hat_new_[num_rob] = dt * (-this->ka_ * ev * this->v_est_) + this->b_hat_old_[num_rob];
        // this->b_hat_new_[num_rob] = find_sign(this->b_hat_new_[num_rob]) * min(abs(this->b_hat_new_[num_rob]), this->b_bound_);

        // from v 8.1 (Matlab)
        // cout<<"Applying Adaptive vel ctrl"<<endl; /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        this->vm_new_[num_rob] = dt * (-this->km_*(this->vm_old_[num_rob] - this->vd_[num_rob])) + this->vm_old_[num_rob];
        double ev = this->v_est_ - this->vm_new_[num_rob];
        this->a_hat_new_[num_rob] = dt * (-this->ka_ * ev * this->v_est_) + this->a_hat_old_[num_rob];
        this->a_hat_new_[num_rob] = find_max(this->a_hat_new_[num_rob], 0.0);
        this->b_hat_new_[num_rob] = dt * (-this->ka_ * ev * this->v_est_) + this->b_hat_old_[num_rob];
        this->b_hat_new_[num_rob] = find_max(this->b_hat_new_[num_rob], 0.0);

}

double apply_pid_ctrl(int num_rob) // PID controller
{
        // cout << "dt = " << dt <<endl;

        double v_req = this->gain_ * find_norm(this->ctrl_[2*num_rob], this->ctrl_[2*num_rob +1]);
        outfile << endl << "v_req = " << v_req;
        outfile << endl << "v_est = " << v_est_;
        double error = v_req - this->speed_gain_* this->v_est_;
        outfile << endl << "error = " << error;

        this->error_sum_[num_rob] = this->error_sum_[num_rob] + error;
        outfile << endl << "error_sum[num_rob]= " << this->error_sum_[num_rob] <<endl;

        double v_temp = this->kp_ * error + this->ki_ * this->error_sum_[num_rob];
        return v_temp;
}

void update_old_new_variables()   // set store xxx_new_ varaibles in xxx_old_
{
        for (int i = 0; i < this->num_rob_; ++i)
        {
                this->tht0_old_[i] = this->tht0_new_[i];
                this->a_hat_old_[i] = this->a_hat_new_[i];
                this->b_hat_old_[i] = this->b_hat_new_[i];
                this->vm_old_[i] = this->vm_new_[i];
                this->thtc_old_[i] = this->thtc_[i];
        }
}

void updata_history (int num_rob) // update position history
{
        if (idx != -1)
        {

                push_back_history(num_rob);
                this->pos_world_history_[num_rob](0,this->mem_) = this->t_new_;
                this->pos_world_history_[num_rob](1,this->mem_) = this->pos_world_new_.data[2*num_rob];
                this->pos_world_history_[num_rob](2,this->mem_) = this->pos_world_new_.data[2*num_rob+1];

                this->pos_image_history_[num_rob](0,this->mem_) = this->t_new_;
                this->pos_image_history_[num_rob](1,this->mem_) = this->pos_image_new_.data[2*num_rob];
                this->pos_image_history_[num_rob](2,this->mem_) = this->pos_image_new_.data[2*num_rob+1];
        }
}


void push_back_history(int num_rob)
{
        for (int i = 0; i < this->mem_; ++i)
        {
                pos_world_history_[num_rob](0,i) = pos_world_history_[num_rob](0,i+1);
                pos_world_history_[num_rob](1,i) = pos_world_history_[num_rob](1,i+1);
                pos_world_history_[num_rob](2,i) = pos_world_history_[num_rob](2,i+1);

                pos_image_history_[num_rob](0,i) = pos_image_history_[num_rob](0,i+1);
                pos_image_history_[num_rob](1,i) = pos_image_history_[num_rob](1,i+1);
                pos_image_history_[num_rob](2,i) = pos_image_history_[num_rob](2,i+1);

        }
}

// helper functions
double find_norm(double val0, double val1)   // find length using euclidean distance rule
{
        double norm_val = sqrt(pow(val0,2) + pow(val1,2));
        return norm_val;
}

double find_max(double val0, double val1) // find max of two numbers
{
        if (val0 > val1)
        {
                return val0;
        }
        else
        {
                return val1;
        }

}

double ang_diff(double thtc, double thte) // find thte-thtc in interval [-pi, pi]
{
        double thtc_r, thte_r;
        thtc_r = thtc /180 * M_PI; // angle in radian
        thte_r = thte /180 * M_PI; // angle in radian
        double diff = thte_r - thtc_r; // angle in radian
        while (diff > M_PI || diff < -M_PI)
        {
                if (diff < -M_PI)
                {
                        diff +=2*M_PI; // angle in radian// angle in radian
                }
                else // (diff > M_PI)
                {

                        diff -= 2*M_PI; // angle in radian

                }

        }
        diff = diff *180 / M_PI;  // angle becomes in degrees
        return diff; // angle in degrees
}

template <typename T>
int find_sign(T val) // find sign of a number
{
        return (T(0) <  val) - (val < T(0));
}
// double find_sign(double val) // find sign of a number
// {
//         if (val > 0)
//         {
//                 return 1;
//         }
//         else
//         {
//                 if (val < 0)
//                 {
//                         return -1;
//                 }
//                 else
//                 {
//                         return 0;
//                 }
//         }
// }

double bound (double val) // bound the angle (val) between 0 and 360 degrees
{
        while (val > 360 || val < 0)
        {
                if (val > 360)
                {
                        val = val-360;
                }
                else // (val < 0)
                {
                        val = val + 360;
                }
        }

        return val;
}

};


int main(int argc, char **argv)
{
        ros::init(argc, argv, "form_ctrl_node");
        ros::NodeHandle nh;
        Ctrl c;
        ros::spinOnce();
        // continue spinning until world and image position data is fetched
        while (c.get_pos_world_new().data.empty() || c.get_pos_image_new().data.empty())
        {
                ros::spinOnce();
        }
        // at this point we have data in pos_world_new_ and pos_image_new_
        ros::spinOnce();
        for(;;++itr)
        {
                ros::spinOnce();
                if(!ros::ok())
                {
                        return 0;
                }
                c.form_ctrl(); // apply formation control
        }
        ros::shutdown();
}
