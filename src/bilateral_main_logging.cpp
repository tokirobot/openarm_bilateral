// Copyright 2025 Reazon Holdings, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <csignal>  
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <fstream>
#include <threads.h>
#include <iomanip>  
#include <exception>
#include <ctime>    

//video logging
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include "dmmotor/damiao_port.hpp"
#include "controller/control.hpp"
#include "controller/dynamics.hpp"
#include "controller/global.hpp"

#define LOGHERTS 30
#define LOGTIME 10

class LeaderNode: public rclcpp::Node
{
        Control *control_l_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::string arm_type_;
        std::string control_type_;

        public:
        LeaderNode(Control *control_l, std::string arm_type, std::string control_type, const rclcpp::NodeOptions& options): Node("LeaderNode", options), control_l_(control_l), arm_type_(arm_type), control_type_(control_type)
        {
                std::vector<double> Dn, Jn, gn, Kp, Kd, Kf, Fc, k, Fv, Fo;

                this->declare_parameter("Dn", std::vector<double>{});
                this->get_parameter("Dn", Dn);

                this->declare_parameter("Jn", std::vector<double>{});
                this->get_parameter("Jn", Jn);

                this->declare_parameter("gn", std::vector<double>{});
                this->get_parameter("gn", gn);

                this->declare_parameter("Kp", std::vector<double>{});
                this->get_parameter("Kp", Kp);

                this->declare_parameter("Kd", std::vector<double>{});
                this->get_parameter("Kd", Kd);

                this->declare_parameter("Kf", std::vector<double>{});
                this->get_parameter("Kf", Kf);

                this->declare_parameter("Fc", std::vector<double>{});
                this->get_parameter("Fc", Fc);

                this->declare_parameter("k", std::vector<double>{});
                this->get_parameter("k", k);

                this->declare_parameter("Fv", std::vector<double>{});
                this->get_parameter("Fv", Fv);

                this->declare_parameter("Fo", std::vector<double>{});
                this->get_parameter("Fo", Fo);

                control_l_->Configure(Dn.data(), Jn.data(), gn.data(), Kp.data(), Kd.data(), Kf.data(), Fc.data(), k.data(), Fv.data(), Fo.data());


                double Ts = 1.0 / FREQUENCY;
                //int msec = (int) (Ts * 1000);

                //timer_ = this->create_wall_timer(
                                //std::chrono::milliseconds(msec),
                                //std::bind(&LeaderNode::timer_callback, this));
                timer_ = this->create_wall_timer(
                                std::chrono::duration<double>(1.0 / FREQUENCY),
                                std::bind(&LeaderNode::timer_callback, this));
                RCLCPP_INFO(this->get_logger(), "LeaderNode initialized");
        }

        ~LeaderNode()
        {
                control_l_->Shutdown();
        }

        private:
        void timer_callback()
        {

                if(control_type_ == "bilate"){
                        control_l_->DoControl();
                }
                else if(control_type_ == "unilate"){
                        control_l_->DoControl_u();
                }
        }
};

class FollowerNode: public rclcpp::Node
{
        Control *control_f_;
        std::string arm_type_;
        std::string control_type_;
        rclcpp::TimerBase::SharedPtr timer_;

        public:
        FollowerNode(Control *control_f, std::string arm_type, std::string control_type, const rclcpp::NodeOptions& options) : Node("FollowerNode", options), control_f_(control_f), arm_type_(arm_type), control_type_(control_type)
        {
                std::vector<double> Dn, Jn, gn, Kp, Kd, Kf, Fc, k, Fv, Fo;

                this->declare_parameter("Dn", std::vector<double>{});
                this->get_parameter("Dn", Dn);

                this->declare_parameter("Jn", std::vector<double>{});
                this->get_parameter("Jn", Jn);

                this->declare_parameter("gn", std::vector<double>{});
                this->get_parameter("gn", gn);

                this->declare_parameter("Kp", std::vector<double>{});
                this->get_parameter("Kp", Kp);

                this->declare_parameter("Kd", std::vector<double>{});
                this->get_parameter("Kd", Kd);

                this->declare_parameter("Kf", std::vector<double>{});
                this->get_parameter("Kf", Kf);

                this->declare_parameter("Fc", std::vector<double>{});
                this->get_parameter("Fc", Fc);

                this->declare_parameter("k", std::vector<double>{});
                this->get_parameter("k", k);

                this->declare_parameter("Fv", std::vector<double>{});
                this->get_parameter("Fv", Fv);

                this->declare_parameter("Fo", std::vector<double>{});
                this->get_parameter("Fo", Fo);

                control_f_->Configure(Dn.data(), Jn.data(), gn.data(), Kp.data(), Kd.data(), Kf.data(), Fc.data(), k.data(), Fv.data(), Fo.data());

                double Ts = 1.0 / FREQUENCY;
                //int msec = (int) (Ts * 1000);

                //timer_ = this->create_wall_timer(
                                //std::chrono::milliseconds(msec),
                                //std::bind(&FollowerNode::timer_callback, this));

                timer_ = this->create_wall_timer(
                                std::chrono::duration<double>(1.0 / FREQUENCY),
                                std::bind(&FollowerNode::timer_callback, this));

                RCLCPP_INFO(this->get_logger(), "FollowerNode initialized");

        }

        ~FollowerNode()
        {
                control_f_->Shutdown();
        }

        private:
        void timer_callback()
        {

                if(control_type_ == "bilate"){
                        control_f_->DoControl();
                }
                else if(control_type_ == "unilate"){
                        control_f_->DoControl_u();

                }
        }
};


class AdminNode: public rclcpp::Node
{
        Control *control_l_;
        Control *control_f_;

        rclcpp::TimerBase::SharedPtr timer_;

        sensor_msgs::msg::JointState response_l;
        sensor_msgs::msg::JointState response_f;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_l_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_f_;

        std::ofstream log_file_l_;
        std::ofstream log_file_f_;
        double time_log_ = 0.0;  
        const double Ts = 1.0/FREQUENCY;

        int ncounts = 0;
        int logging_sample_time = (int)FREQUENCY /LOGHERTS;
        int log_stop_num = LOGHERTS * LOGTIME;
        int data_num = 0;
        bool log_finished = false;

        public:
        AdminNode(Control *control_l, Control *control_f) : Node("AdminNode"), control_l_(control_l), control_f_(control_f)
        {
                setup_all_publishers();
                if(LOGGING){
                        std::string base_log_dir = std::string(std::getenv("HOME")) + "/ros2_ws/src/openarm_bilateral/data/";

                        auto t = std::time(nullptr);
                        auto tm = *std::localtime(&t);
                        std::ostringstream oss_base;
                        oss_base << base_log_dir << "/" << std::put_time(&tm, "%Y%m%d_%H%M%S");
                        std::string full_log_dir = oss_base.str();

                        std::string motion_dir = full_log_dir + "/action";
                        rcpputils::fs::create_directories(rcpputils::fs::path(motion_dir));

                        std::ostringstream oss_l, oss_f;
                        oss_l << motion_dir << "/action_l.csv";
                        oss_f << motion_dir << "/action_f.csv";
                        log_file_l_.open(oss_l.str());
                        log_file_f_.open(oss_f.str());

                        if(!log_file_l_.is_open() || !log_file_f_.is_open()) {
                                RCLCPP_ERROR(this->get_logger(), "failed to open log files!");
                        } else {
                                log_file_l_ << "time";
                                log_file_f_ << "time";
                                for (int i = 0; i < NJOINTS; ++i) log_file_l_ << ",qpos" << i;
                                for (int i = 0; i < NJOINTS; ++i) log_file_l_ << ",qvel" << i;
                                for (int i = 0; i < NJOINTS; ++i) log_file_l_ << ",qeff" << i;
                                for (int i = 0; i < NJOINTS; ++i) log_file_f_ << ",qpos" << i;
                                for (int i = 0; i < NJOINTS; ++i) log_file_f_ << ",qvel" << i;
                                for (int i = 0; i < NJOINTS; ++i) log_file_f_ << ",qeff" << i;
                                log_file_l_ << std::endl;
                                log_file_f_ << std::endl;
                        }
                }

                double Ts = 1.0 / FREQUENCY;
                //int msec = (int) (Ts * 1000);
                timer_ = this->create_wall_timer(
                                std::chrono::duration<double>(Ts),
                                std::bind(&AdminNode::timer_callback, this));
                //timer_ = this->create_wall_timer(
                //std::chrono::milliseconds(msec),
                //std::bind(&AdminNode::timer_callback, this));
                RCLCPP_INFO(this->get_logger(), "FREQUENCY=%f, logging_sample_time=%d, log_stop_num=%d", FREQUENCY, logging_sample_time, log_stop_num);

                RCLCPP_INFO(this->get_logger(), "AdminNode initialized");
        }

        ~AdminNode()
        {
                if(log_file_l_.is_open()) log_file_l_.close();
                if(log_file_f_.is_open()) log_file_f_.close();
        }
        private:
        void setup_all_publishers(void)
        {
                pub_l_ = this->create_publisher<sensor_msgs::msg::JointState>(
                                "openarm/leader/joint_state", 1
                                );
                pub_f_ = this->create_publisher<sensor_msgs::msg::JointState>(
                                "openarm/follower/joint_state", 1
                                );
        }

        void timer_callback()
        {

                control_f_->GetResponse(&response_f);
                control_l_->GetResponse(&response_l);

                for (int i = 0; i < NJOINTS; ++i) {
                        response_f.effort[i] *= BETA;
                        response_l.position[i] *= ALPHA;
                        response_l.velocity[i] *= ALPHA;
                }

                //for (int i = 0; i < NJOINTS; ++i) {
                //// Leader arm
                //if (response_l.position[i] < position_limit_min_L[i] || response_l.position[i] > position_limit_max_L[i]) {
                //std::cerr << "[WARN] L pos[" << i << "] out of limit: " << response_l.position[i] << std::endl;
                //}

                //// Follower arm
                //if (response_f.position[i] < position_limit_min_F[i] || response_f.position[i] > position_limit_max_F[i]) {
                //std::cerr << "[WARN] F pos[" << i << "] out of limit: " << response_f.position[i] << std::endl;
                //}
                //}               

                for (int i = 0; i < NJOINTS; ++i) {
                        response_l.position[i] = std::clamp(response_l.position[i], position_limit_min_L[i], position_limit_max_L[i]);
                        //response_l.velocity[i] = std::clamp(response_l.velocity[i], -velocity_limit_l[i],  velocity_limit_l[i]);
                        //response_l.effort[i]   = std::clamp(response_l.effort[i],   -effort_limit_l[i],    effort_limit_l[i]);

                        response_f.position[i] = std::clamp(response_f.position[i], position_limit_min_F[i], position_limit_max_F[i]);
                        //response_f.velocity[i] = std::clamp(response_f.velocity[i], -velocity_limit_F[i],  velocity_limit_F[i]);
                        //response_f.effort[i]   = std::clamp(response_f.effort[i],   -effort_limit_F[i],    effort_limit_F[i]);
                }

                control_f_->SetReference(&response_l);
                control_l_->SetReference(&response_f);


                pub_l_->publish(*control_l_->response_);
                pub_f_->publish(*control_f_->response_); 

                // logging
                ++ncounts;
                int new_data_num = ncounts / logging_sample_time;
                //std::cout << "new_data_num" << new_data_num << std::endl;
                //std::cout << "ncounts" << ncounts << std::endl;
                if (LOGGING && !log_finished && new_data_num > data_num) {
                        data_num = new_data_num;

                        time_log_ += Ts * logging_sample_time; 
                        log_file_l_ << time_log_;
                        for (int i = 0; i < NJOINTS; ++i) {
                                log_file_l_ << "," << response_l.position[i];
                        }
                        for (int i = 0; i < NJOINTS; ++i) {
                                log_file_l_ << "," << response_l.velocity[i];
                        }
                        for (int i = 0; i < NJOINTS; ++i) {
                                log_file_l_ << "," << response_l.effort[i];
                        }
                        log_file_l_ << std::endl;

                        log_file_f_ << time_log_;
                        for (int i = 0; i < NJOINTS; ++i) {
                                log_file_f_ << "," << response_f.position[i];
                        }
                        for (int i = 0; i < NJOINTS; ++i) {
                                log_file_f_ << "," << response_f.velocity[i];
                        }
                        for (int i = 0; i < NJOINTS; ++i) {
                                log_file_f_ << "," << response_f.effort[i];
                        }
                        log_file_f_ << std::endl;


                        if(data_num >= log_stop_num) {
                                log_file_l_.close();
                                log_file_f_.close();
                                log_finished = true;
                                std::cout << "LOGGING COMPLETE!!  " << data_num << " sample recorded." << std::endl;
                        }
                }
        }

};


class RealSenseLogging : public rclcpp::Node
{
        public:
                RealSenseLogging()
                        : Node("realsense_logging_node")
                {
                        using namespace std::chrono_literals;



                        auto t = std::time(nullptr);
                        auto tm = *std::localtime(&t);
                        std::ostringstream oss_base;
                        oss_base << std::string(std::getenv("HOME")) << "/ros2_ws/src/openarm_bilateral/data/"
                                << std::put_time(&tm, "%Y%m%d_%H%M%S");
                        std::string base_dir = oss_base.str();

                        // Save to base_dir + /observations/cam_xxx
                        save_dir_high_ = base_dir + "/observations/cam_high";
                        save_dir_wrist_ = base_dir + "/observations/cam_right_wrist";

                        rcpputils::fs::create_directories(rcpputils::fs::path(save_dir_high_));
                        rcpputils::fs::create_directories(rcpputils::fs::path(save_dir_wrist_));

                        rs2::context ctx;
                        auto list = ctx.query_devices();
                        if (list.size() == 0) {
                                RCLCPP_FATAL(this->get_logger(), "No RealSense devices detected.");
                                throw std::runtime_error("No RealSense devices found.");
                        }

                        // serial number of realsense 
                        std::string cam_high = "922612071413";
                        std::string cam_right_wrist = "215222079832";


                        for (auto&& dev : list) {
                                std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                                RCLCPP_INFO(this->get_logger(), "Detected RealSense with serial: %s", serial.c_str());

                                if (!pipe_high_ && serial == cam_high) {
                                        cfg_high_.enable_device(serial);
                                        cfg_high_.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
                                        pipe_high_ = std::make_unique<rs2::pipeline>();
                                        pipe_high_->start(cfg_high_);
                                        RCLCPP_INFO(this->get_logger(), "Started CAM_HIGH: %s", serial.c_str());
                                }
                                else if (!pipe_wrist_ && serial == cam_right_wrist) {
                                        cfg_wrist_.enable_device(serial);
                                        cfg_wrist_.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
                                        pipe_wrist_ = std::make_unique<rs2::pipeline>();
                                        pipe_wrist_->start(cfg_wrist_);
                                        RCLCPP_INFO(this->get_logger(), "Started CAM_WRIST: %s", serial.c_str());
                                }
                        }

                        

                        if (!pipe_high_ && !pipe_wrist_) {
                                RCLCPP_FATAL(this->get_logger(), "No known cameras (CAM_HIGH or CAM_WRIST) found.");
                                throw std::runtime_error("No known cameras found.");
                        }

                        double lograte = 30.0;
                        timer_ = this->create_wall_timer(
                                        std::chrono::duration<double>(1.0 / lograte),
                                        std::bind(&RealSenseLogging::timer_callback, this)
                                        );
                }

        private:
                void save_frame(const rs2::video_frame &frame, const std::string &dir)
                {
                        if (!frame || !frame.get_data()) return;

                        int width = frame.get_width();
                        int height = frame.get_height();
                        cv::Mat image(cv::Size(width, height), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
                        if (image.empty()) return;

                        cv::Mat bgr_image;
                        cv::cvtColor(image, bgr_image, cv::COLOR_RGB2BGR);

                        auto now = std::chrono::high_resolution_clock::now();
                        auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
                        double timestamp_sec = static_cast<double>(now_ns) / 1e9;

                        std::stringstream filename;
                        filename << dir << "/" << std::fixed << std::setprecision(4) << timestamp_sec << ".jpg";
                        cv::imwrite(filename.str(), bgr_image);
                }

                void timer_callback()
                {
                        if (log_finished) {
                                return;
                        }

                        try {
                                if (pipe_high_) {
                                        auto frames = pipe_high_->wait_for_frames();
                                        save_frame(frames.get_color_frame(), save_dir_high_);
                                }
                                if (pipe_wrist_) {
                                        auto frames = pipe_wrist_->wait_for_frames();
                                        save_frame(frames.get_color_frame(), save_dir_wrist_);
                                }

                                ++ncounts;
                                if (ncounts >= log_stop_num && !log_finished) {
                                        log_finished = true;
                                        RCLCPP_INFO(this->get_logger(), "RealSense logging stopped after %d frames (%.1f seconds).",
                                                        ncounts, ncounts / static_cast<float>(LOGHERTS));
                                }

                        } catch (const rs2::error &e) {
                                RCLCPP_WARN(this->get_logger(), "RealSense error: %s", e.what());
                        }
                }


                std::unique_ptr<rs2::pipeline> pipe_high_;
                std::unique_ptr<rs2::pipeline> pipe_wrist_;
                rs2::config cfg_high_, cfg_wrist_;
                std::string save_dir_high_, save_dir_wrist_;
                rclcpp::TimerBase::SharedPtr timer_;


                int ncounts = 0;
                int log_stop_num = LOGHERTS * LOGTIME;
                int data_num = 0;
                bool log_finished = false;

};


int main(int argc, char **argv)
{
        printOpenArmBanner();

        rclcpp::init(argc,argv);
        bool arm_type_valid = false;
        bool control_mode_valid = false;
        std::string arm_type;
        std::string control_mode;
        for (int i = 1; i < argc; i++) {
                std::string arg = argv[i];
                if(arg == "--ros-args") break;
                if (arg == "right_arm" || arg == "left_arm") {
                        arm_type = arg;
                        arm_type_valid = true;
                } else if (arg == "bilate" || arg == "unilate") {
                        control_mode = arg;
                        control_mode_valid = true;
                } else {
                        RCLCPP_ERROR(rclcpp::get_logger("main"), "Invalid argument: %s", arg.c_str());
                        rclcpp::shutdown();
                        return 1;
                }
        }

        if (!arm_type_valid) {
                RCLCPP_WARN(rclcpp::get_logger("main"), "No arm_type specified. Using default 'left_arm'.");
        }
        if (!control_mode_valid) {
                RCLCPP_WARN(rclcpp::get_logger("main"), "No control_mode specified. Using default 'bilate'.");
        }
        std::string can_dev_l;
        std::string can_dev_f;

        // if left or right
        if(arm_type == "left_arm"){
                can_dev_l = CAN2;
                can_dev_f = CAN3;
        }
        else if(arm_type == "right_arm")
        {
                can_dev_l = CAN0;
                can_dev_f = CAN1;
        }

        std::cout << "[INFO] Leader CAN device type: " << can_dev_l << std::endl;
        std::cout << "[INFO] Follower CAN device type: " << can_dev_f << std::endl;

        if (AUTOBALANCER_ON) {
                std::cout << "[CONFIG] AutoBalancer is ENABLED" << std::endl;
        } else {
                std::cout << "[CONFIG] AutoBalancer is DISABLED" << std::endl;
        }

        if (LOGGING) {
                std::cout << "[CONFIG] Logging is ENABLED" << std::endl;
        } else {
                std::cout << "[CONFIG] Logging is DISABLED" << std::endl;
        }


        DamiaoPort* arm_l = new DamiaoPort(can_dev_l, 
                        {DM_Motor_Type::DM4340, DM_Motor_Type::DM4340, DM_Motor_Type::DM4340, DM_Motor_Type::DM4340,
                        DM_Motor_Type::DM4310, DM_Motor_Type::DM4310, DM_Motor_Type::DM4310, DM_Motor_Type::DM4310},
                        {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08},
                        {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18},
                        {true, true, true, true, true, true, true, true},
                        Control_Type::MIT,
                        CAN_MODE_FD
                        );

        DamiaoPort* arm_f = new DamiaoPort(can_dev_f, 
                        {DM_Motor_Type::DM4340, DM_Motor_Type::DM4340, DM_Motor_Type::DM4340, DM_Motor_Type::DM4340,
                        DM_Motor_Type::DM4310, DM_Motor_Type::DM4310, DM_Motor_Type::DM4310, DM_Motor_Type::DM4310},
                        {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08},
                        {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18},
                        {true, true, true, true, true, true, true, true},
                        Control_Type::MIT,
                        CAN_MODE_FD
                        );

        // use if you need unique zero position
        //arm_l->setZeroPosition();
        //arm_f->setZeroPosition();

        Control* control_l = new Control(arm_l, 1.0 / FREQUENCY, ROLE_LEADER, arm_type);
        Control* control_f = new Control(arm_f, 1.0 / FREQUENCY, ROLE_FOLLOWER, arm_type);

        //control_l->Configure(Dn_L, Gn_L, Jn_L, gn_L, Kp_L, Kd_L, Kf_L, Fc_L, k_L, Fv_L, Fo_L);
        //control_f->Configure(Dn_F, Gn_F, Jn_F, gn_F, Kp_F, Kd_F, Kf_F, Fc_F, k_F, Fv_F, Fo_F);

        if (!control_l->Setup()) {
                return 1;
        }
        if (!control_f->Setup()) {
                return 1;
        }

        std::vector<double> init_pos = {0, 0, 0, 0, 0, 0, 0, 0};
        std::vector<double> kp = {2.0 ,2.0 ,2.0 ,2.0 ,2.0 ,2.0 ,0.0, 0.0};
        std::vector<double> kd = {0.1 ,0.1 ,0.1 ,0.1 ,0.1 ,0.1 ,0.0, 0.0};


        arm_l->setGoalPositionsSync(init_pos, kp, kd);
        arm_f->setGoalPositionsSync(init_pos, kp, kd);

        std::vector<double> positions_l = arm_l->getPositions();
        std::vector<double> velocities_l = arm_l->getVelocities();

        std::vector<double> positions_f = arm_f->getPositions();
        std::vector<double> velocities_f = arm_f->getVelocities();

        for(int i = 0; i < NMOTORS ; i++)
        {
                control_l->response_->position[i] = positions_l[i];
                control_f->response_->position[i] = positions_f[i];
        }

        sleep(0.5);

        control_l->SetReference(control_f->response_);
        control_f->SetReference(control_l->response_);

        //set home postion
        std::thread thread_l(&Control::AdjustPosition, control_l);
        std::thread thread_f(&Control::AdjustPosition, control_f);

        thread_l.join();
        thread_f.join();

        control_l->SetReference(control_f->response_);
        control_f->SetReference(control_l->response_);

        rclcpp::executors::MultiThreadedExecutor exec;
        rclcpp::NodeOptions options;

        rclcpp::on_shutdown([&]() {
                        std::cout << "ROS shutdown triggered" << std::endl;
                        control_l->Shutdown();
                        control_f->Shutdown();
                        });


        auto leader_node = std::make_shared<LeaderNode>(control_l,arm_type, control_mode, options);
        auto follower_node = std::make_shared<FollowerNode>(control_f, arm_type, control_mode, options);
        auto admin_node = std::make_shared<AdminNode>(control_l, control_f);

        auto realsense_node = std::make_shared<RealSenseLogging>();
       

        RCLCPP_INFO(rclcpp::get_logger("main"), "DO NOT MOVE THE ARM!!!!!!!");
        RCLCPP_INFO(rclcpp::get_logger("main"), "Warming up RealSense...");
        rclcpp::sleep_for(std::chrono::seconds(3));
        RCLCPP_INFO(rclcpp::get_logger("main"), "Warmup complete.");

        exec.add_node(admin_node);
        exec.add_node(follower_node);
        exec.add_node(leader_node);
        exec.add_node(realsense_node);
        exec.spin();

        rclcpp::shutdown();
        return 0;

}


