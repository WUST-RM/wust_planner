#pragma once

#include "path_searching/kinodynamic_astar.hpp"
#include "plan_env/edt_environment.hpp"
#include "bspline/non_uniform_bspline.hpp"
#include "planner_manager/mpc.hpp"
#include "bspline_opt/bspline_optimizer.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/convert.h>              
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//#include "rmoss_interfaces/msg/chassis_cm_dmind.hpp"
//#include "rmoss_interfaces/msg/gimbal_cmd.hpp"
namespace fast_planner
{
    enum FSM_EXEC_STATE
    {
        INIT,
        WAIT_TARGET,
        GEN_NEW_TRAJ,
        REPLAN_TRAJ,
        EXEC_TRAJ,
        REPLAN_NEW
    };

    struct PlanParameters
    {
        /* planning algorithm parameters */
        double max_vel_, max_acc_, max_jerk_; // physical limits
        double local_traj_len_;               // local replanning trajectory length
        double ctrl_pt_dist;                  // distance between adjacient B-spline
                                              // control points
        double clearance_;
        int dynamic_;
        /* processing time */
        double time_search_ = 0.0;
        double time_optimize_ = 0.0;
        double time_adjust_ = 0.0;
    };

    class kino_replan_fsm : public rclcpp::Node
    {
    public:
        kino_replan_fsm(const rclcpp::NodeOptions &options) : Node("planner_manager", options)
        {
            mpc_ptr.reset(new Mpc);
            mpc_ptr->init(std::shared_ptr<rclcpp::Node>(this));

            pp_.dynamic_ = this->declare_parameter<int>("manager.dynamic_environment", 0);
            pp_.ctrl_pt_dist = this->declare_parameter<double>("manager.control_points_distance", 0.5);
            pp_.max_vel_ = this->declare_parameter<double>("manager.max_vel", 3.0);
            pp_.max_acc_ = this->declare_parameter<double>("manager.max_acc", 1.0);
            pp_.max_jerk_ = this->declare_parameter<double>("manager.max_jerk", 2.);
            bspline_optimizers_.resize(3);
            base_frame_ = this->declare_parameter<std::string>("manager.base_frame", "");
            std::string odom_topic = this->declare_parameter<std::string>("manager.odometry", std::string("/odom"));
            initial_pose_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                odom_topic, 10, std::bind(&kino_replan_fsm::initialPoseCallback, this, std::placeholders::_1));

            std::string goal_topic = this->declare_parameter<std::string>("manager.goal_topic", std::string("/goal_pose"));
            // 订阅目标位置 (goal) 话题
            goal_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                goal_topic, 10, std::bind(&kino_replan_fsm::goalPoseCallback, this, std::placeholders::_1));

            sdf_map_.reset(new SDFMap);
            sdf_map_->initMap(std::shared_ptr<rclcpp::Node>(this));

            edt_environment_.reset(new EDTEnvironment);
            edt_environment_->setMap(sdf_map_);

            kino_path_finder_.reset(new KinodynamicAstar);
            kino_path_finder_->setParam(std::shared_ptr<rclcpp::Node>(this));
            kino_path_finder_->setEnvironment(edt_environment_);
            kino_path_finder_->init();
            for (int i = 0; i < 3; ++i)
            {
                bspline_optimizers_[i].reset(new BsplineOptimizer);
                bspline_optimizers_[i]->setParam(std::shared_ptr<rclcpp::Node>(this));
                bspline_optimizers_[i]->setEnvironment(edt_environment_);
            }
            int control_cmd_frequency = this->declare_parameter<int>("manager.control_cmd_frequency", 100);
            CarType = this->declare_parameter<int>("manager.CarType", 0);
            control_cmd_pub = this->create_wall_timer(std::chrono::duration<double>(1.0 / control_cmd_frequency), std::bind(&kino_replan_fsm::publish_control_cmd, this));
            path_pub_ = this->create_publisher<nav_msgs::msg::Path>("robot_path", 10);
            visited_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("visited_node", 10);
            predict_path_pub = this->create_publisher<nav_msgs::msg::Path>("predict_path", 10);
            std::string cmd_vel_topic = this->declare_parameter<std::string>("manager.cmd_vel", std::string("/cmd_vel"));

            // cmd_vel_pub = this->create_publisher<rmoss_interfaces::msg::ChassisCMDmind>(cmd_vel_topic, 10);
            cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
            // cmd_Gimbal_pub = this->create_publisher<rmoss_interfaces::msg::GimbalCmd>("/red_standard_robot1/robot_base/gimbal_cmd", 10);
            // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            int safety_cheak_frequency = this->declare_parameter<int>("manager.safety_cheak_frequency", 20);

            safety_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / safety_cheak_frequency), std::bind(&kino_replan_fsm::checkCollisionCallback, this));
            // optimis = this->create_wall_timer(0.5s, std::bind(&kino_replan_fsm::trajOptimize, this));
            int plan_cheak_frequency = this->declare_parameter<int>("manager.plan_cheak", 20);
            control_output = this->declare_parameter<bool>("manager.control_output", false);

            plan_timer = this->create_wall_timer(std::chrono::duration<double>(1.0 / plan_cheak_frequency), std::bind(&kino_replan_fsm::plan_timer_callback, this));
            //  tf = this->create_wall_timer(0.01s, std::bind(&kino_replan_fsm::tf_callback, this));
            tf_buffer_ =
                std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ =
                std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            double l = this->declare_parameter<double>("manager.Car_L", 0.5);
            double w = this->declare_parameter<double>("manager.Car_W", 0.45);
            safe_check_distance = this->declare_parameter<double>("manager.safe_check_distance", 3.0);
            Car_radius = l;
            double half_l = l / 2.0;
            double half_w = w / 2.0;

            // 局部坐标系的四个顶点
            local_points = {
                {-half_l, -half_w},
                {half_l, -half_w},
                {half_l, half_w},
                {-half_l, half_w}};

            is_inited= true;
               
        }
        ~kino_replan_fsm()
        {
            // 1. 取消定时器，防止析构后回调访问已析构内存
            if (control_cmd_pub)    control_cmd_pub->cancel();
            if (safety_timer_)      safety_timer_->cancel();
            if (plan_timer)         plan_timer->cancel();
            if (tf)                 tf->cancel();
            if (optimis)            optimis->cancel();
            

            // 2. 清空 B-spline 优化器列表
            bspline_optimizers_.clear();

            // 3. 重置资源指针，触发对应对象析构
            mpc_ptr.reset();             // MPC 控制器
            sdf_map_.reset();            // SDF 地图
            edt_environment_.reset();    // EDT 环境
            kino_path_finder_.reset();   // 路径搜索器
            tf_listener_.reset();        // TF 监听器
            tf_buffer_.reset();          // TF 缓冲区
            path_pub_.reset();
            visited_pub.reset();
            predict_path_pub.reset();
            cmd_vel_pub.reset();
           // cmd_Gimbal_pub.reset();
          
            RCLCPP_INFO(this->get_logger(), "kino_replan_fsm destructed");

            // 4. 其余 RCLCPP 发布者/订阅者会随 Node 基类析构自动清理，无需显式处理
        }

        std::vector<Eigen::Vector2d> local_points;
        bool init_initialPose, trigger_=false;
        Eigen::Vector2d init_pose, end_pt_;
        Eigen::Vector2d odom_vel_; // odometry state
        Eigen::Quaterniond odom_orient_;
        SDFMap::Ptr sdf_map_;
        EDTEnvironment::Ptr edt_environment_;
        unique_ptr<KinodynamicAstar> kino_path_finder_;
        bool control_output = false;
        int CarType = 0;
        double Car_radius = 0.5;
        void trajOptimize()
        {
            if (exec_state_ != EXEC_TRAJ)
                return;
            int cost_function = BsplineOptimizer::NORMAL_PHASE;

            auto ctrl_pts = bspline_optimizers_[2]->BsplineOptimizeTraj(position_traj_.getControlPoint(), position_traj_.getInterval(), cost_function, 1, 1);

            NonUniformBspline pos = NonUniformBspline(ctrl_pts, 3, position_traj_.getInterval());
            pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);

            bool feasible = pos.checkFeasibility(false);
            int iter_num = 0;
            while (!feasible && rclcpp::ok())
            {
                feasible = pos.reallocateTime();

                if (++iter_num >= 3)
                    break;
            }
            position_traj_ = pos;
            velocity_traj_ = position_traj_.getDerivative();
            acceleration_traj_ = velocity_traj_.getDerivative();
            duration_ = position_traj_.getTimeSum();
            planYaw(start_yaw_, target_yaw);

            bspline_process(10086);
        }
    
//         void initialPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
// {

//     geometry_msgs::msg::TransformStamped tf_map_odom;
//     try {
//         tf_map_odom = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
//     } catch (const tf2::TransformException &ex) {
//         RCLCPP_ERROR(get_logger(), "fail to lookup map→odom: %s", ex.what());
//         init_initialPose = false;
//         return;
//     }

//     geometry_msgs::msg::TransformStamped tf_odom_gimbal;
//     try {
//         tf_odom_gimbal = tf_buffer_->lookupTransform("odom", "gimbal_yaw", tf2::TimePointZero);
//     } catch (const tf2::TransformException &ex) {
//         RCLCPP_ERROR(get_logger(), "fail to lookup odom→gimbal_yaw_fake: %s", ex.what());
//         init_initialPose = false;
//         return;
//     }


//     Eigen::Affine3f T_map_odom = Eigen::Affine3f::Identity();
//     Eigen::Vector3f t_map_odom(tf_map_odom.transform.translation.x,
//                                tf_map_odom.transform.translation.y,
//                                tf_map_odom.transform.translation.z);
//     Eigen::Quaternionf q_map_odom(tf_map_odom.transform.rotation.w,
//                                   tf_map_odom.transform.rotation.x,
//                                   tf_map_odom.transform.rotation.y,
//                                   tf_map_odom.transform.rotation.z);
//     T_map_odom.translate(t_map_odom);
//     T_map_odom.rotate(q_map_odom);

//     Eigen::Affine3f T_odom_gimbal = Eigen::Affine3f::Identity();
//     Eigen::Vector3f t_odom_gimbal(tf_odom_gimbal.transform.translation.x,
//                                   tf_odom_gimbal.transform.translation.y,
//                                   tf_odom_gimbal.transform.translation.z);
//     Eigen::Quaternionf q_odom_gimbal(tf_odom_gimbal.transform.rotation.w,
//                                      tf_odom_gimbal.transform.rotation.x,
//                                      tf_odom_gimbal.transform.rotation.y,
//                                      tf_odom_gimbal.transform.rotation.z);
//     T_odom_gimbal.translate(t_odom_gimbal);
//     T_odom_gimbal.rotate(q_odom_gimbal);

//     Eigen::Affine3f T_map_gimbal = T_map_odom * T_odom_gimbal;


//     Eigen::Vector3f pos_map = T_map_gimbal.translation();
//     init_pose(0) = pos_map.x();
//     init_pose(1) = pos_map.y();

//     Eigen::Matrix3f rot_map = T_map_gimbal.rotation();
//     float yaw_map = std::atan2(rot_map(1, 0), rot_map(0, 0));
//     start_yaw_(0) = yaw_map;


//     start_yaw_(1) = msg->twist.twist.angular.z;
//     odom_vel_(0)  = msg->twist.twist.linear.x;
//     odom_vel_(1)  = msg->twist.twist.linear.y;

//     tf2::Quaternion q_gimbal(tf_odom_gimbal.transform.rotation.x,
//                               tf_odom_gimbal.transform.rotation.y,
//                               tf_odom_gimbal.transform.rotation.z,
//                               tf_odom_gimbal.transform.rotation.w);
//     double roll_g, pitch_g, yaw_g;
//     tf2::Matrix3x3(q_gimbal).getRPY(roll_g, pitch_g, yaw_g);
//     gimbal_rpy << roll_g, pitch_g, yaw_g;

//     tf2::Quaternion q_map(tf_map_odom.transform.rotation.x,
//                            tf_map_odom.transform.rotation.y,
//                            tf_map_odom.transform.rotation.z,
//                            tf_map_odom.transform.rotation.w);
//     double roll_m, pitch_m, yaw_m;
//     tf2::Matrix3x3(q_map).getRPY(roll_m, pitch_m, yaw_m);
//     odom_rpy << roll_m, pitch_m, yaw_m;

//     init_initialPose = true;


// }
void initialPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{   if(!is_inited) return;
  // 1. map→odom
  geometry_msgs::msg::TransformStamped tf_map_odom;
  try {
    tf_map_odom = tf_buffer_->lookupTransform(
      "map", "odom", tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_INFO(get_logger(), "fail to lookup map→odom: %s", ex.what());
    init_initialPose = false;
    return;
  }
  tf2::Transform t_map_odom;
  tf2::fromMsg(tf_map_odom.transform, t_map_odom);
  double roll_m, pitch_m, yaw_m;
  tf2::Matrix3x3(t_map_odom.getRotation()).getRPY(roll_m, pitch_m, yaw_m);

  // 2. odom→gimbal_yaw
  geometry_msgs::msg::TransformStamped tf_odom_gimbal;
  try {
    tf_odom_gimbal = tf_buffer_->lookupTransform(
      "odom", base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_INFO(get_logger(), "fail to lookup odom→%s: %s", base_frame_.c_str(), ex.what());
    init_initialPose = false;
    return;
  }
  tf2::Transform t_odom_gimbal;
  tf2::fromMsg(tf_odom_gimbal.transform, t_odom_gimbal);
  double roll_g, pitch_g, yaw_g;
  tf2::Matrix3x3(t_odom_gimbal.getRotation()).getRPY(roll_g, pitch_g, yaw_g);

  // —— 以下是你原来对 map→gimbal 的处理（可选） ——
  tf2::Transform t_map_gimbal = t_map_odom * t_odom_gimbal;
  tf2::Vector3 pos = t_map_gimbal.getOrigin();
  init_pose(0) = pos.x();
  init_pose(1) = pos.y();
  double roll, pitch, yaw;
  tf2::Matrix3x3(t_map_gimbal.getRotation()).getRPY(roll, pitch, yaw);
  start_yaw_(0) = static_cast<float>(yaw);
  tf2::Quaternion t_q = {
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w};
    double roll_o, pitch_o, yaw_o;
    tf2::Matrix3x3(t_q).getRPY(roll_o, pitch_o, yaw_o);

  // 保存各自的 RPY 到成员变量
   odom_rpy << roll_g, pitch_g, yaw_g;    
   gimbal_rpy << roll_g, pitch_g, yaw_g;  

  // 里程计速度、偏航角速度
  start_yaw_(1) = msg->twist.twist.angular.z;
  odom_vel_(0)  = msg->twist.twist.linear.x;
  odom_vel_(1)  = msg->twist.twist.linear.y;

  init_initialPose = true;
}

        
        
        // void initialPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
        // {
        //     geometry_msgs::msg::TransformStamped t;
        //     try
        //     {
        //         // 获取 gimbal_yaw 相对于 odom 的变换（注意方向）
        //         t = tf_buffer_->lookupTransform("gimbal_yaw", "odom", tf2::TimePointZero);
        //     }
        //     catch (const tf2::TransformException &ex)
        //     {
        //         RCLCPP_ERROR(this->get_logger(), "Could not transform odom to gimbal_yaw: %s", ex.what());
        //         init_initialPose = false;
        //         return;
        //     }

        //     // 提取旋转矩阵
        //     tf2::Quaternion tf_q(
        //         t.transform.rotation.x,
        //         t.transform.rotation.y,
        //         t.transform.rotation.z,
        //         t.transform.rotation.w);
        //     tf2::Matrix3x3 rot(tf_q); // R_gimbal^odom

        //     // ====== 转换位置（直接从 tf 中得到）======
        //     init_pose(0) = t.transform.translation.x;
        //     init_pose(1) = t.transform.translation.y;

        //     // ====== 转换角度 ======
        //     double roll, pitch, yaw;
        //     rot.getRPY(roll, pitch, yaw);
        //     gimbal_rpy << roll, pitch, yaw;
        //     start_yaw_(0) = yaw;

        //     // ====== 转换角速度到 gimbal_yaw 坐标系 ======
        //     tf2::Vector3 ang_vel_odom(
        //         msg->twist.twist.angular.x,
        //         msg->twist.twist.angular.y,
        //         msg->twist.twist.angular.z);
        //     tf2::Vector3 ang_vel_gimbal = rot * ang_vel_odom;
        //     start_yaw_(1) = ang_vel_gimbal.z(); // 只用 yaw 方向角速度

        //     // ====== 转换线速度到 gimbal_yaw 坐标系 ======
        //     tf2::Vector3 lin_vel_odom(
        //         msg->twist.twist.linear.x,
        //         msg->twist.twist.linear.y,
        //         msg->twist.twist.linear.z);
        //     tf2::Vector3 lin_vel_gimbal = rot * lin_vel_odom;
        //     odom_vel_(0) = lin_vel_gimbal.x();
        //     odom_vel_(1) = lin_vel_gimbal.y();

        //     // ====== 保存 odom 的原始 rpy（可选）======
        //     tf2::Quaternion odom_q(
        //         msg->pose.pose.orientation.x,
        //         msg->pose.pose.orientation.y,
        //         msg->pose.pose.orientation.z,
        //         msg->pose.pose.orientation.w);
        //     tf2::Matrix3x3(odom_q).getRPY(roll, pitch, yaw);
        //     odom_rpy << roll, pitch, yaw;

        //     init_initialPose = true;
        // }


        Eigen::Vector3d odom_rpy;
        Eigen::Vector3d gimbal_rpy;
        void checkCollisionCallback()
        {
            // LocalTrajData *info = &planner_manager_->local_data_;

            if (have_traj)
            {

                auto &edt_env = edt_environment_;

                double dist = pp_.dynamic_ ? edt_env->evaluateCoarseEDT(end_pt_, /* time to program start + */ duration_) : edt_env->evaluateCoarseEDT(end_pt_, -1.0);

                if (dist <= 0.3)
                {
                    /* try to find a max distance goal around */
                    bool new_goal = false;
                    const double dr = 0.27, dtheta = 30;
                    double new_x, new_y, max_dist = -1.0;
                    Eigen::Vector2d goal;

                    for (double r = dr; r <= 5 * dr + 1e-3; r += dr)
                    {
                        for (double theta = -90; theta <= 270; theta += dtheta)
                        {

                            new_x = end_pt_(0) + r * cos(theta / 57.3);
                            new_y = end_pt_(1) + r * sin(theta / 57.3);

                            Eigen::Vector2d new_pt(new_x, new_y);
                            dist = pp_.dynamic_ ? edt_env->evaluateCoarseEDT(new_pt, /* time to program start+ */ duration_) : edt_env->evaluateCoarseEDT(new_pt, -1.0);

                            if (dist > max_dist)
                            {
                                /* reset end_pt_ */
                                goal(0) = new_x;
                                goal(1) = new_y;
                                max_dist = dist;
                            }
                        }
                    }

                    if (max_dist > 0.3)
                    {
                        cout << "change goal, replan." << endl;
                        end_pt_ = goal;
                        have_traj = true;
                        // end_vel_.setZero();

                        // if (exec_state_ == EXEC_TRAJ)
                        // {
                        //     changeFSMExecState(REPLAN_TRAJ, "SAFETY");
                        // }
                        if (exec_state_ == EXEC_TRAJ)
                            exec_state_ = REPLAN_TRAJ;
                        // visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
                    }
                    else
                    {
                        // have_target_ = false;
                        // cout << "Goal near collision, stop." << endl;
                        // changeFSMExecState(WAIT_TARGET, "SAFETY");
                        cout << "goal near collision, keep retry" << endl;
                        // changeFSMExecState(REPLAN_TRAJ, "FSM");
                        exec_state_ = REPLAN_TRAJ;

                        // std_msgs::Empty emt;
                        // replan_pub_.publish(emt);
                    }
                }
            }

            /* ---------- check trajectory ---------- */
            if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ)
            {
                double dist;
                bool safe = checkTrajCollision(dist);

                if (!safe)
                {
                    // cout << "current traj in collision." << endl;
                    RCLCPP_WARN(this->get_logger(), "current traj in collision.");
                    // changeFSMExecState(REPLAN_TRAJ, "SAFETY");
                    exec_state_ = REPLAN_TRAJ;
                }
            }
        }
        bool mode = 0;
        int nihao = 0;

        void plan_timer_callback()
        {
            // std::cout << "start_yaw_" << start_yaw_ << std::endl;
            static int fsm_num = 0;
            fsm_num++;
            if (fsm_num == 100)
            {
                if (!init_initialPose)
                    cout << "no odom." << endl;
                if (!trigger_)
                    cout << "wait for goal." << endl;
                fsm_num = 0;
            }
            switch (exec_state_)
            {
            case INIT:
            {
                if (!init_initialPose)
                {
                    return;
                }
                if (!trigger_)
                {
                    return;
                }
              

                exec_state_ = GEN_NEW_TRAJ;

                break;
            }
            case WAIT_TARGET:
            {
                have_traj = false;
                trigger_ = false;
                // if (nihao < 6)
                // {
                //     end_pt_ = {std::get<0>(arr[nihao]).pose.position.x, std::get<0>(arr[nihao]).pose.position.y};
                //     mode = std::get<1>(arr[nihao]);
                //     edt_environment_->sdf_map_->setLocalMap(std::get<2>(arr[nihao]));
                //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
                //     nihao++;
                //     trigger_ = true;
                // }
                geometry_msgs::msg::Twist twist;
                // rmoss_interfaces::msg::ChassisCMDmind cmd_;
                twist.linear.x = 0;
                twist.linear.y = 0;
                twist.angular.z = 0;
                // cmd_.twist = twist;
                // cmd_.mode = 1;
                if (control_output)
                    cmd_vel_pub->publish(twist);
                if (!trigger_)
                    return;
                else
                {
                    exec_state_ = GEN_NEW_TRAJ;
                }
                break;
            }
            case EXEC_TRAJ:
            {
                auto pre_position = position_traj_.evaluateDeBoorT((rclcpp::Clock().now() - start_time).seconds());
                if (collision_replan_count > 5)
                {
                    collision_replan_count = 0;
                    position_traj_ = Last_position_traj;
                    position_traj_.reallocateTime(1.0);
                    bspline_process(1.0);
                    break;
                }
                if ((init_pose - pre_position).norm() > 1.0)
                {
                    exec_state_ = GEN_NEW_TRAJ;
                    collision_replan_count++;
                    RCLCPP_WARN(this->get_logger(), "RePlan Cause of Path No Execute");
                }

                break;
            }
            case GEN_NEW_TRAJ:
            {
                bool success = plan(init_pose, odom_vel_, Eigen::Vector2d(0, 0), end_pt_, Eigen::Vector2d(0, 0));

                if (success)
                {
                    exec_state_ = EXEC_TRAJ;
                }
                else
                {
                    exec_state_ = GEN_NEW_TRAJ;
                }
                break;
            }
            case REPLAN_TRAJ:
            {

                auto position = position_traj_.evaluateDeBoorT((rclcpp::Clock().now() - start_time).seconds());
                auto vel = velocity_traj_.evaluateDeBoorT((rclcpp::Clock().now() - start_time).seconds());
                // odom_vel_ = velocity_traj_.evaluateDeBoorT((rclcpp::Clock().now() - start_time).seconds());
                auto acc = acceleration_traj_.evaluateDeBoorT((rclcpp::Clock().now() - start_time).seconds());

                bool success = plan(position, vel, acc, end_pt_, Eigen::Vector2d(0, 0));
                if (success)
                {
                    exec_state_ = EXEC_TRAJ;
                }
                else
                {
                    exec_state_ = GEN_NEW_TRAJ;
                }
                break;
            }
            }
            if (exec_state_ == EXEC_TRAJ && (end_pt_ - init_pose).norm() < 0.2 && (odom_vel_).norm() < 0.1)
            {
                Last_position_traj = position_traj_;
                exec_state_ = WAIT_TARGET;
            }
        }
        int collision_replan_count = 0;
        bool plan(Eigen::Vector2d position, Eigen::Vector2d vel_, Eigen::Vector2d acc_, Eigen::Vector2d end_pt, Eigen::Vector2d end_vel);
        std::tuple<geometry_msgs::msg::PoseStamped, bool, int> arr[6];
        void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {   
            
            end_pt_ << msg->pose.position.x, msg->pose.position.y;


            RCLCPP_INFO(this->get_logger(), "Received goal pose: position=(%.2f, %.2f), orientation=(%.2f, %.2f, %.2f, %.2f)",
                        msg->pose.position.x, msg->pose.position.y,
                        msg->pose.orientation.x, msg->pose.orientation.y,
                        msg->pose.orientation.z, msg->pose.orientation.w);
            tf2::Quaternion t_q(
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z,
                msg->pose.orientation.w);
            double roll_temp, pitch_temp;
            tf2::Matrix3x3(t_q).getRPY(roll_temp, pitch_temp, target_yaw);

            trigger_ = true;
            // std::cout << "exec_state_" <<  exec_state_ << std::endl;
            if (exec_state_ == EXEC_TRAJ)
                exec_state_ = REPLAN_TRAJ;
            else if (exec_state_ == WAIT_TARGET)
                exec_state_ = GEN_NEW_TRAJ;
            // std::cout << "exec_state_" <<  exec_state_ << std::endl;
        }
        // void goalPoseCallback(
        //     const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        //   {
        //       // 1. 拉取最新的 map->odom 变换（忽略任何时间戳）
        //       geometry_msgs::msg::TransformStamped tf_stamped;
        //       try {
        //           tf_stamped = tf_buffer_->lookupTransform(
        //               "odom",               // 目标坐标系
        //               msg->header.frame_id, // 源坐标系
        //               tf2::TimePointZero);  // 时间零：强制取最新
        //       }
        //       catch (const tf2::TransformException &ex) {
        //           RCLCPP_WARN(this->get_logger(),
        //               "获取最新变换失败: 从 %s 到 odom: %s",
        //               msg->header.frame_id.c_str(), ex.what());
        //           return;
        //       }
          
        //       // 2. 将原始 PoseStamped（连同它的 header）不做任何时间修改，应用到 odom
        //       geometry_msgs::msg::PoseStamped goal_odom;
        //       tf2::doTransform(*msg, goal_odom, tf_stamped);
        //       end_pt_ << goal_odom.pose.position.x, goal_odom.pose.position.y;
          
        //       // 3. 提取位置
        //       double x = goal_odom.pose.position.x;
        //       double y = goal_odom.pose.position.y;
          
        //       // 4. 提取 yaw
        //       tf2::Quaternion q(
        //           goal_odom.pose.orientation.x,
        //           goal_odom.pose.orientation.y,
        //           goal_odom.pose.orientation.z,
        //           goal_odom.pose.orientation.w);
        //       double roll, pitch, yaw;
        //       tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
          
        //       // 5. 日志与状态机触发
        //       RCLCPP_INFO(this->get_logger(),
        //           "在 odom 坐标系下，新目标: position=(%.2f, %.2f), yaw=%.2f",
        //           x, y, yaw);
          
        //       trigger_ = true;
        //       if (exec_state_ == EXEC_TRAJ) {
        //           exec_state_ = REPLAN_TRAJ;
        //       } else if (exec_state_ == WAIT_TARGET) {
        //           exec_state_ = GEN_NEW_TRAJ;
        //       }
        //   }
        // void goalPoseCallback(
        //     const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        // {
        //     // 0. 原始目标先变换到 odom（同之前逻辑）
        //     geometry_msgs::msg::TransformStamped tf_src_odom;
        //     try {
        //         tf_src_odom = tf_buffer_->lookupTransform(
        //             "odom",                   // 目标：odom
        //             msg->header.frame_id,     // 源：msg->header.frame_id
        //             tf2::TimePointZero        // 最新可用
        //         );
        //     }
        //     catch (const tf2::TransformException &ex) {
        //         RCLCPP_WARN(get_logger(),
        //             "无法获取 %s->odom 变换: %s",
        //             msg->header.frame_id.c_str(), ex.what());
        //         return;
        //     }
        //     geometry_msgs::msg::PoseStamped goal_odom;
        //     tf2::doTransform(*msg, goal_odom, tf_src_odom);
        
        //     // 1. 拉取最新的 map->odom 变换
        //     geometry_msgs::msg::TransformStamped tf_map_odom;
        //     try {
        //         tf_map_odom = tf_buffer_->lookupTransform(
        //             "map",                // 源：map
        //             "odom",               // 目标：odom
        //             tf2::TimePointZero    // 最新可用
        //         );
        //     }
        //     catch (const tf2::TransformException &ex) {
        //         RCLCPP_WARN(get_logger(),
        //             "无法获取 map->odom 变换: %s", ex.what());
        //         return;
        //     }
        
        //     // 2. 计算 odom->map 的逆变换
        //     tf2::Transform t_map_odom;
        //     tf2::fromMsg(tf_map_odom.transform, t_map_odom);
        //     tf2::Transform t_odom_map = t_map_odom.inverse();
        
        //     geometry_msgs::msg::TransformStamped tf_odom_map;
        //     tf_odom_map.header.stamp    = tf_map_odom.header.stamp;
        //     tf_odom_map.header.frame_id = "odom";
        //     tf_odom_map.child_frame_id  = "map";
        //     tf_odom_map.transform       = tf2::toMsg(t_odom_map);
        
        //     // 3. 将 goal_odom 再转换到 map，从而“减去”里程计漂移
        //     geometry_msgs::msg::PoseStamped goal_map;
        //     tf2::doTransform(goal_odom, goal_map, tf_odom_map);
        
        //     // 4. 更新终点并提取 yaw
        //     end_pt_ << goal_map.pose.position.x,
        //                goal_map.pose.position.y;
        
        //     tf2::Quaternion q(
        //         goal_map.pose.orientation.x,
        //         goal_map.pose.orientation.y,
        //         goal_map.pose.orientation.z,
        //         goal_map.pose.orientation.w);
        //     double roll, pitch, yaw;
        //     tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        //     RCLCPP_INFO(get_logger(),
        //         "在 map 坐标系下，新目标: position=(%.2f, %.2f), yaw=%.2f",
        //         end_pt_[0], end_pt_[1], yaw);
        
        //     trigger_ = true;
        //     if (exec_state_ == EXEC_TRAJ) {
        //         exec_state_ = REPLAN_TRAJ;
        //     } else if (exec_state_ == WAIT_TARGET) {
        //         exec_state_ = GEN_NEW_TRAJ;
        //     }
        // }
        
          
          

        // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        FSM_EXEC_STATE exec_state_ = INIT;
        double target_yaw = 0;
        bool have_traj = false;
        bool is_inited = false;
        NonUniformBspline position_traj_, velocity_traj_, acceleration_traj_, yaw_traj_, yawdot_traj_;
        vector<NonUniformBspline> traj;
        NonUniformBspline Last_position_traj;
        double traj_duration, duration_;
        double safe_check_distance;
        // int N;
        // double dt;
        rclcpp::Time start_time;
        std::unique_ptr<Mpc> mpc_ptr;
        Eigen::Vector3d current_state;
        Eigen::Vector2d start_yaw_, start_acc_;
        void publish_control_cmd();
        void bspline_process(double t);
        bool checkTrajCollision(double &distance);

        void planYaw(const Eigen::Vector2d &start_yaw, const double end_yaw_set);
        rclcpp::TimerBase::SharedPtr control_cmd_pub, tf, safety_timer_, plan_timer, optimis;
        PlanParameters pp_;
        vector<BsplineOptimizer::Ptr> bspline_optimizers_;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, predict_path_pub;
        // rclcpp::Publisher<rmoss_interfaces::msg::ChassisCMDmind>::SharedPtr cmd_vel_pub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
       // rclcpp::Publisher<rmoss_interfaces::msg::GimbalCmd>::SharedPtr cmd_Gimbal_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr visited_pub;
        sensor_msgs::msg::PointCloud2 cloud_msg___;

        nav_msgs::msg::Path path_, predict_path;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr initial_pose_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscription_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        std::string base_frame_;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}