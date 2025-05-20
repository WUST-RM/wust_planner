#include "bspline_opt/bspline_optimizer.hpp"
#include <iostream>
#include <nlopt.hpp>
#include <OsqpEigen/OsqpEigen.h>  // 确保你使用的是 osqp-eigen 包
namespace fast_planner
{
    const int BsplineOptimizer::SMOOTHNESS = (1 << 0);
    const int BsplineOptimizer::DISTANCE = (1 << 1);
    const int BsplineOptimizer::FEASIBILITY = (1 << 2);
    const int BsplineOptimizer::ENDPOINT = (1 << 3);
    const int BsplineOptimizer::GUIDE = (1 << 4);
    const int BsplineOptimizer::WAYPOINTS = (1 << 6);

    const int BsplineOptimizer::GUIDE_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::GUIDE;
    const int BsplineOptimizer::NORMAL_PHASE =
        BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE | BsplineOptimizer::FEASIBILITY;

    void BsplineOptimizer::setEnvironment(const EDTEnvironment::Ptr &env)
    {
        this->edt_environment_ = env;
    }

    void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
    {
        control_points_ = points;
        dim_ = control_points_.cols();
    }
    void BsplineOptimizer::setWaypoints(const vector<Eigen::Vector2d> &waypts,
                                        const vector<int> &waypt_idx)
    {
        waypoints_ = waypts;
        waypt_idx_ = waypt_idx;
    }
    void BsplineOptimizer::setBsplineInterval(const double &ts) { bspline_interval_ = ts; }

    void BsplineOptimizer::setTerminateCond(const int &max_num_id, const int &max_time_id)
    {
        max_num_id_ = max_num_id;
        max_time_id_ = max_time_id;
    }
    void BsplineOptimizer::setCostFunction(const int &cost_code)
    {
        cost_function_ = cost_code;

        // print optimized cost function
        string cost_str;
        if (cost_function_ & SMOOTHNESS)
            cost_str += "smooth |";
        if (cost_function_ & DISTANCE)
            cost_str += " dist  |";
        if (cost_function_ & FEASIBILITY)
            cost_str += " feasi |";
        if (cost_function_ & ENDPOINT)
            cost_str += " endpt |";
        if (cost_function_ & GUIDE)
            cost_str += " guide |";
        if (cost_function_ & WAYPOINTS)
            cost_str += " waypt |";

        RCLCPP_INFO_STREAM(node_->get_logger(), "cost func: " << cost_str);

       
    }
    void BsplineOptimizer::setParam(std::shared_ptr<rclcpp::Node> nh)
    {
        node_ = nh;
        if (node_->has_parameter("optimization.jerk_smoothness_weight"))
        {
            lambda1_ = node_->get_parameter("optimization.jerk_smoothness_weight").as_double();
            lambda2_ = node_->get_parameter("optimization.distance_weight").as_double();
            lambda3_ = node_->get_parameter("optimization.feasibility_weight").as_double();
            lambda4_ = node_->get_parameter("optimization.end_point_weight").as_double();
            lambda5_ = node_->get_parameter("optimization.guide_cost_weight").as_double();
            lambda6_ = node_->get_parameter("optimization.visibility_cost_weight").as_double();
            lambda7_ = node_->get_parameter("optimization.waypoints_cost_weight").as_double();
            lambda8_ = node_->get_parameter("optimization.acc_smoothness").as_double();

            // nh.param("optimization/dist0", dist0_, -1.0);
            dist0_ = node_->get_parameter("optimization.dist0").as_double();

            max_vel_ = node_->get_parameter("optimization.max_vel").as_double();
            max_acc_ = node_->get_parameter("optimization.max_acc").as_double();
            // nh.param("optimization/visib_min", visib_min_, -1.0);
            // nh.param("optimization/dlmin", dlmin_, -1.0);
            // nh.param("optimization/wnl", wnl_, -1.0);

            max_iteration_num_[0] = node_->get_parameter("optimization.max_iteration_num1").as_int();
            max_iteration_num_[1] = node_->get_parameter("optimization.max_iteration_num2").as_int();
            max_iteration_num_[2] = node_->get_parameter("optimization.max_iteration_num3").as_int();
            max_iteration_num_[3] = node_->get_parameter("optimization.max_iteration_num4").as_int();

            max_iteration_time_[0] = node_->get_parameter("optimization.max_iteration_time1").as_double();
            max_iteration_time_[1] = node_->get_parameter("optimization.max_iteration_time2").as_double();
            max_iteration_time_[2] = node_->get_parameter("optimization.max_iteration_time3").as_double();
            max_iteration_time_[3] = node_->get_parameter("optimization.max_iteration_time4").as_double();

            algorithm1_ = node_->get_parameter("optimization.quadratic_cost").as_int();
            algorithm2_ = node_->get_parameter("optimization.general_cost").as_int();

            order_ = node_->get_parameter("optimization.order").as_int();
            return;
        }

        lambda1_ = node_->declare_parameter<double>("optimization.jerk_smoothness_weight", 10.0);
        lambda2_ = node_->declare_parameter<double>("optimization.distance_weight", 8.0);
        lambda3_ = node_->declare_parameter<double>("optimization.feasibility_weight", 0.0001);
        lambda4_ = node_->declare_parameter<double>("optimization.end_point_weight", 0.05);
        lambda5_ = node_->declare_parameter<double>("optimization.guide_cost_weight", -1.0);
        lambda6_ = node_->declare_parameter<double>("optimization.visibility_cost_weight", -1.0);
        lambda7_ = node_->declare_parameter<double>("optimization.waypoints_cost_weight", 100.0);
        lambda8_ = node_->declare_parameter<double>("optimization.acc_smoothness", -1.0);

        // nh.param("optimization/dist0", dist0_, -1.0);
        dist0_ = node_->declare_parameter<double>("optimization.dist0", 0.6);

        max_vel_ = node_->declare_parameter<double>("optimization.max_vel", 3.0);
        max_acc_ = node_->declare_parameter<double>("optimization.max_acc", 1.0);
        // nh.param("optimization/visib_min", visib_min_, -1.0);
        // nh.param("optimization/dlmin", dlmin_, -1.0);
        // nh.param("optimization/wnl", wnl_, -1.0);

        max_iteration_num_[0] = node_->declare_parameter<int>("optimization.max_iteration_num1", 2);
        max_iteration_num_[1] = node_->declare_parameter<int>("optimization.max_iteration_num2", 300);
        max_iteration_num_[2] = node_->declare_parameter<int>("optimization.max_iteration_num3", 200);
        max_iteration_num_[3] = node_->declare_parameter<int>("optimization.max_iteration_num4", 200);

        max_iteration_time_[0] = node_->declare_parameter<double>("optimization.max_iteration_time1", 0.0001);
        max_iteration_time_[1] = node_->declare_parameter<double>("optimization.max_iteration_time2", 0.005);
        max_iteration_time_[2] = node_->declare_parameter<double>("optimization.max_iteration_time3", 0.003);
        max_iteration_time_[3] = node_->declare_parameter<double>("optimization.max_iteration_time4", 0.003);

        algorithm1_ = node_->declare_parameter<int>("optimization.quadratic_cost", 11);
        algorithm2_ = node_->declare_parameter<int>("optimization.general_cost", 15);

        order_ = node_->declare_parameter<int>("optimization.order", 3);
    }
    Eigen::MatrixXd BsplineOptimizer::BsplineOptimizeTraj(const Eigen::MatrixXd &points, const double &ts,
                                                          const int &cost_function, int max_num_id,
                                                          int max_time_id)
    {
        setControlPoints(points);
        setBsplineInterval(ts);
        setCostFunction(cost_function);
        setTerminateCond(max_num_id, max_time_id);

        optimize();
        return this->control_points_;
    }
    void BsplineOptimizer::optimize()
    {
        /* initialize solver */
        iter_num_ = 0;
        min_cost_ = std::numeric_limits<double>::max();
        const int pt_num = control_points_.rows();
        g_q_.resize(pt_num);
        g_smoothness_.resize(pt_num);
        g_distance_.resize(pt_num);
        g_feasibility_.resize(pt_num);
        g_endpoint_.resize(pt_num);
        g_waypoints_.resize(pt_num);
        g_guide_.resize(pt_num);

        if (cost_function_ & ENDPOINT)
        {
            variable_num_ = dim_ * (pt_num - order_);
            // end position used for hard constraint
            end_pt_ = (1 / 6.0) *
                      (control_points_.row(pt_num - 3) + 4 * control_points_.row(pt_num - 2) +
                       control_points_.row(pt_num - 1));
        }
        else
        {
            variable_num_ = max(0, dim_ * (pt_num - 2 * order_));
        }

        /* do optimization using NLopt slover */
        nlopt::opt opt(nlopt::algorithm(isQuadratic() ? algorithm1_ : algorithm2_), variable_num_);
        opt.set_min_objective(BsplineOptimizer::costFunction, this);
        opt.set_maxeval(max_iteration_num_[max_num_id_]);
        opt.set_maxtime(max_iteration_time_[max_time_id_]);
        opt.set_xtol_rel(1e-5);

        vector<double> q(variable_num_);
        for (int i = order_; i < pt_num; ++i)
        {
            if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_)
                continue;
            for (int j = 0; j < dim_; j++)
            {
                q[dim_ * (i - order_) + j] = control_points_(i, j);
            }
        }

        if (dim_ != 1)
        {
            vector<double> lb(variable_num_), ub(variable_num_);
            const double bound = 10.0;
            for (int i = 0; i < variable_num_; ++i)
            {
                lb[i] = q[i] - bound;
                ub[i] = q[i] + bound;
            }
            opt.set_lower_bounds(lb);
            opt.set_upper_bounds(ub);
        }

        try
        {
            // cout << fixed << setprecision(7);
            // vec_time_.clear();
            // vec_cost_.clear();
            // time_start_ = ros::Time::now();

            double final_cost;
            nlopt::result result = opt.optimize(q, final_cost);

            /* retrieve the optimization result */
            // cout << "Min cost:" << min_cost_ << endl;
        }
        catch (std::exception &e)
        {
            RCLCPP_WARN(node_->get_logger(), "[Optimization]: nlopt exception");
            cout << e.what() << endl;
        }
        for (int i = order_; i < control_points_.rows(); ++i)
        {
            if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_)
                continue;
            for (int j = 0; j < dim_; j++)
            {
                control_points_(i, j) = best_variable_[dim_ * (i - order_) + j];
            }
        }

        // if (!(cost_function_ & GUIDE))
        //     RCLCPP_INFO_STREAM(node_->get_logger(), "iter num: " << iter_num_);
    }
    void BsplineOptimizer::combineCost(const std::vector<double> &x, std::vector<double> &grad,
                                       double &f_combine)
    {
        /* convert the NLopt format vector to control points. */

        // This solver can support 1D-3D B-spline optimization, but we use Vector3d to store each control point
        // For 1D case, the second and third elements are zero, and similar for the 2D case.
        for (int i = 0; i < order_; i++)
        {
            for (int j = 0; j < dim_; ++j)
            {
                g_q_[i][j] = control_points_(i, j);
            }
            for (int j = dim_; j < 2; ++j)
            {
                g_q_[i][j] = 0.0;
            }
        }

        for (int i = 0; i < variable_num_ / dim_; i++)
        {
            for (int j = 0; j < dim_; ++j)
            {
                g_q_[i + order_][j] = x[dim_ * i + j];
            }
            for (int j = dim_; j < 2; ++j)
            {
                g_q_[i + order_][j] = 0.0;
            }
        }

        if (!(cost_function_ & ENDPOINT))
        {
            for (int i = 0; i < order_; i++)
            {

                for (int j = 0; j < dim_; ++j)
                {
                    g_q_[order_ + variable_num_ / dim_ + i][j] =
                        control_points_(control_points_.rows() - order_ + i, j);
                }
                for (int j = dim_; j < 2; ++j)
                {
                    g_q_[order_ + variable_num_ / dim_ + i][j] = 0.0;
                }
            }
        }

        f_combine = 0.0;
        grad.resize(variable_num_);
        fill(grad.begin(), grad.end(), 0.0);

        /*  evaluate costs and their gradient  */
        double f_smoothness, f_distance, f_feasibility, f_endpoint, f_guide, f_waypoints;
        f_smoothness = f_distance = f_feasibility = f_endpoint = f_guide = f_waypoints = 0.0;

        if (cost_function_ & SMOOTHNESS)
        {
            calcSmoothnessCost(g_q_, f_smoothness, g_smoothness_);
            f_combine += lambda1_ * f_smoothness;
            for (int i = 0; i < variable_num_ / dim_; i++)
                for (int j = 0; j < dim_; j++)
                    grad[dim_ * i + j] += lambda1_ * g_smoothness_[i + order_](j);
        }
        if (cost_function_ & DISTANCE)
        {
            calcDistanceCost(g_q_, f_distance, g_distance_);
            f_combine += lambda2_ * f_distance;
            for (int i = 0; i < variable_num_ / dim_; i++)
                for (int j = 0; j < dim_; j++)
                    grad[dim_ * i + j] += lambda2_ * g_distance_[i + order_](j);
        }
        if (cost_function_ & FEASIBILITY)
        {
            calcFeasibilityCost(g_q_, f_feasibility, g_feasibility_);
            f_combine += lambda3_ * f_feasibility;
            for (int i = 0; i < variable_num_ / dim_; i++)
                for (int j = 0; j < dim_; j++)
                    grad[dim_ * i + j] += lambda3_ * g_feasibility_[i + order_](j);
        }
        if (cost_function_ & ENDPOINT)
        {
            calcEndpointCost(g_q_, f_endpoint, g_endpoint_);
            f_combine += lambda4_ * f_endpoint;
            for (int i = 0; i < variable_num_ / dim_; i++)
                for (int j = 0; j < dim_; j++)
                    grad[dim_ * i + j] += lambda4_ * g_endpoint_[i + order_](j);
        }
        if (cost_function_ & GUIDE)
        {
            calcGuideCost(g_q_, f_guide, g_guide_);
            f_combine += lambda5_ * f_guide;
            for (int i = 0; i < variable_num_ / dim_; i++)
                for (int j = 0; j < dim_; j++)
                    grad[dim_ * i + j] += lambda5_ * g_guide_[i + order_](j);
        } // 没用
        if (cost_function_ & WAYPOINTS)
        {
            calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
            f_combine += lambda7_ * f_waypoints;
            for (int i = 0; i < variable_num_ / dim_; i++)
                for (int j = 0; j < dim_; j++)
                    grad[dim_ * i + j] += lambda7_ * g_waypoints_[i + order_](j);
        }
        /*  print cost  */
        // if ((cost_function_ & WAYPOINTS) && iter_num_ % 10 == 0) {
        //   cout << iter_num_ << ", total: " << f_combine << ", acc: " << lambda8_ * f_view
        //        << ", waypt: " << lambda7_ * f_waypoints << endl;
        // }

        // if (optimization_phase_ == SECOND_PHASE) {
        // cout << "cost:"
        //  << ", smooth: " << lambda1_ * f_smoothness
        //  << " , dist:" << lambda2_ * f_distance
        //  << ", fea: " << lambda3_ * f_feasibility << endl;
        // << ", end: " << lambda4_ * f_endpoint
        // << ", guide: " << lambda5_ * f_guide
        // }
    }
    void BsplineOptimizer::calcSmoothnessCost(const vector<Eigen::Vector2d> &q, double &cost,
                                              vector<Eigen::Vector2d> &gradient)
    {
        cost = 0.0;
        Eigen::Vector2d zero(0, 0);
        std::fill(gradient.begin(), gradient.end(), zero);
        Eigen::Vector2d jerk, temp_j;

        for (int i = 0; i < q.size() - order_; i++)
        {
            /* evaluate jerk */
            jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i];
            cost += jerk.squaredNorm();
            temp_j = 2.0 * jerk;
            /* jerk gradient */
            gradient[i + 0] += -temp_j;
            gradient[i + 1] += 3.0 * temp_j;
            gradient[i + 2] += -3.0 * temp_j;
            gradient[i + 3] += temp_j;
        }
    }

    void BsplineOptimizer::calcDistanceCost(const vector<Eigen::Vector2d> &q, double &cost,
                                            vector<Eigen::Vector2d> &gradient)
    {
        cost = 0.0;
        Eigen::Vector2d zero(0, 0);
        std::fill(gradient.begin(), gradient.end(), zero);

        double dist;
        Eigen::Vector2d dist_grad, g_zero(0, 0);

        int end_idx = (cost_function_ & ENDPOINT) ? q.size() : q.size() - order_;

        for (int i = order_; i < end_idx; i++)
        {
            edt_environment_->evaluateEDTWithGrad(q[i], -1.0, dist, dist_grad);
            if (dist_grad.norm() > 1e-4)
                dist_grad.normalize();

            if (dist < dist0_)
            {
                cost += pow(dist - dist0_, 2);
                gradient[i] += 2.0 * (dist - dist0_) * dist_grad;
            }
        }
    }

    void BsplineOptimizer::calcFeasibilityCost(const vector<Eigen::Vector2d> &q, double &cost,
                                               vector<Eigen::Vector2d> &gradient)
    {
        cost = 0.0;
        Eigen::Vector2d zero(0, 0);
        std::fill(gradient.begin(), gradient.end(), zero);

        /* abbreviation */
        double ts, vm2, am2, ts_inv2, ts_inv4;
        vm2 = max_vel_ * max_vel_;
        am2 = max_acc_ * max_acc_;

        ts = bspline_interval_;
        ts_inv2 = 1 / ts / ts;
        ts_inv4 = ts_inv2 * ts_inv2;

        /* velocity feasibility */
        for (int i = 0; i < q.size() - 1; i++)
        {
            Eigen::Vector2d vi = q[i + 1] - q[i];

            for (int j = 0; j < 2; j++)
            {
                double vd = vi(j) * vi(j) * ts_inv2 - vm2;
                if (vd > 0.0)
                {
                    cost += pow(vd, 2);

                    double temp_v = 4.0 * vd * ts_inv2;
                    gradient[i + 0](j) += -temp_v * vi(j);
                    gradient[i + 1](j) += temp_v * vi(j);
                }
            }
        }

        /* acceleration feasibility */
        for (int i = 0; i < q.size() - 2; i++)
        {
            Eigen::Vector2d ai = q[i + 2] - 2 * q[i + 1] + q[i];

            for (int j = 0; j < 2; j++)
            {
                double ad = ai(j) * ai(j) * ts_inv4 - am2;
                if (ad > 0.0)
                {
                    cost += pow(ad, 2);

                    double temp_a = 4.0 * ad * ts_inv4;
                    gradient[i + 0](j) += temp_a * ai(j);
                    gradient[i + 1](j) += -2 * temp_a * ai(j);
                    gradient[i + 2](j) += temp_a * ai(j);
                }
            }
        }
    }

    void BsplineOptimizer::calcEndpointCost(const vector<Eigen::Vector2d> &q, double &cost,
                                            vector<Eigen::Vector2d> &gradient)
    {
        cost = 0.0;
        Eigen::Vector2d zero(0, 0);
        std::fill(gradient.begin(), gradient.end(), zero);

        // zero cost and gradient in hard constraints
        Eigen::Vector2d q_3, q_2, q_1, dq;
        q_3 = q[q.size() - 3];
        q_2 = q[q.size() - 2];
        q_1 = q[q.size() - 1];

        dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - end_pt_;
        cost += dq.squaredNorm();

        gradient[q.size() - 3] += 2 * dq * (1 / 6.0);
        gradient[q.size() - 2] += 2 * dq * (4 / 6.0);
        gradient[q.size() - 1] += 2 * dq * (1 / 6.0);
    }

    void BsplineOptimizer::calcWaypointsCost(const vector<Eigen::Vector2d> &q, double &cost,
                                             vector<Eigen::Vector2d> &gradient)
    {
        cost = 0.0;
        Eigen::Vector2d zero(0, 0);
        std::fill(gradient.begin(), gradient.end(), zero);

        Eigen::Vector2d q1, q2, q3, dq;

        // for (auto wp : waypoints_) {
        for (int i = 0; i < waypoints_.size(); ++i)
        {
            Eigen::Vector2d waypt = waypoints_[i];
            int idx = waypt_idx_[i];

            q1 = q[idx];
            q2 = q[idx + 1];
            q3 = q[idx + 2];

            dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
            cost += dq.squaredNorm();

            gradient[idx] += dq * (2.0 / 6.0);     // 2*dq*(1/6)
            gradient[idx + 1] += dq * (8.0 / 6.0); // 2*dq*(4/6)
            gradient[idx + 2] += dq * (2.0 / 6.0);
        }
    }

    /* use the uniformly sampled points on a geomertic path to guide the
     * trajectory. For each control points to be optimized, it is assigned a
     * guiding point on the path and the distance between them is penalized */
    void BsplineOptimizer::calcGuideCost(const vector<Eigen::Vector2d> &q, double &cost,
                                         vector<Eigen::Vector2d> &gradient)
    {
        cost = 0.0;
        Eigen::Vector2d zero(0, 0);
        std::fill(gradient.begin(), gradient.end(), zero);

        int end_idx = q.size() - order_;

        for (int i = order_; i < end_idx; i++)
        {
            Eigen::Vector2d gpt = guide_pts_[i - order_];
            cost += (q[i] - gpt).squaredNorm();
            gradient[i] += 2 * (q[i] - gpt);
        }
    }
    double BsplineOptimizer::costFunction(const std::vector<double> &x, std::vector<double> &grad,
                                          void *func_data)
    {
        BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
        double cost;
        opt->combineCost(x, grad, cost);
        opt->iter_num_++;

        /* save the min cost result */
        if (cost < opt->min_cost_)
        {
            opt->min_cost_ = cost;
            opt->best_variable_ = x;
        }
        return cost;
    }
    bool BsplineOptimizer::isQuadratic()
    {
        if (cost_function_ == GUIDE_PHASE)
        {
            return true;
        }
        else if (cost_function_ == (SMOOTHNESS | WAYPOINTS))
        {
            return true;
        }
        return false;
    }
//     bool BsplineOptimizer::generateHomotopicCorridor(
//         const std::vector<Eigen::Vector2d>& path_pts,
//         double safety_radius,
//         std::vector<Polyhedron>& corridor)
// {
//     corridor.clear();
//     if (!edt_environment_ || path_pts.size() < 2) return false;

//     // 对每段生成一个 Polyhedron
//     for (size_t i = 0; i + 1 < path_pts.size(); ++i) {
//         const auto &p1 = path_pts[i];
//         const auto &p2 = path_pts[i+1];
//         Eigen::Vector2d mid = 0.5 * (p1 + p2);
//         Eigen::Vector2d dir = (p2 - p1).normalized();

//         // 四个法向（2D）
//         std::array<Eigen::Vector2d,4> normals = {{
//             Eigen::Vector2d(-dir.y(), dir.x()),  // 左法向
//             Eigen::Vector2d( dir.y(),-dir.x()),  // 右法向
//             Eigen::Vector2d( 1, 0),              // x+
//             Eigen::Vector2d(-1, 0)               // x-
//         }};

//         Polyhedron poly;
//         // 利用 edt_environment_ 评估距离
//         for (auto &n : normals) {
//             double lo = 0, hi = safety_radius * 5;
//             for (int it = 0; it < 16; ++it) {
//                 double mid_delta = 0.5 * (lo + hi);
//                 Eigen::Vector2d test_pt = mid + mid_delta * n;
//                 double dist;
//                 Eigen::Vector2d grad;
//                 // 这里使用带梯度的 EDT 评估
//                 edt_environment_->evaluateEDTWithGrad(test_pt, /*time=*/0.0, dist, grad);
//                 // 若与所有障碍（含动态预测）距离大于安全半径，继续扩张
//                 if (dist > safety_radius)
//                     lo = mid_delta;
//                 else
//                     hi = mid_delta;
//             }
//             double δ = lo;
//             // 添加超平面 nᵀ x ≤ nᵀ mid + δ
//             poly.addPlane(n, n.dot(mid) + δ);
//         }
//         corridor.push_back(poly);
//     }
//     return true;
// }
// Eigen::MatrixXd BsplineOptimizer::optimizeWithCorridor(
//     const Eigen::MatrixXd& init_ctrl_pts,
//     double ts,
//     const std::vector<Polyhedron>& corridor)
// {
//     // 1. 构建 cost 项：H 和 f
//     Eigen::SparseMatrix<double> H;
//     Eigen::VectorXd f;

//     buildCost(init_ctrl_pts, ts, H, f);  


//     // 2. 构建不等式约束 A * x <= b
//     std::vector<Eigen::Triplet<double>> A_triplets;
//     std::vector<double> b_vals;
//     int row = 0;

//     const int dim = init_ctrl_pts.rows();    // e.g. 2
//     const int N = init_ctrl_pts.cols();      // 控制点数量
//     for (size_t seg = 0; seg < corridor.size(); ++seg) {
//         Eigen::MatrixXd Ai;
//         Eigen::VectorXd bi;
//         corridor[seg].getConstraints(Ai, bi);  // Ai * x <= bi

//         int num_planes = Ai.rows();

//         for (int i = 0; i < num_planes; ++i) {
//             for (int j = 0; j < dim; ++j) {
//                 int var_idx = seg * dim + j;
//                 A_triplets.emplace_back(row, var_idx, Ai(i, j));
//             }
//             b_vals.push_back(bi(i));
//             row++;
//         }
//     }
//     std::cout<<  "A_triplets.size() = "<<  A_triplets.size()<<  std::endl;

//     Eigen::SparseMatrix<double> A(row, dim * N);
//     std::cout<<  "A"<<  std::endl;
//     A.setFromTriplets(A_triplets.begin(), A_triplets.end());
//     std::cout<<  "set  A"<<  std::endl;
//     Eigen::VectorXd b = Eigen::Map<Eigen::VectorXd>(b_vals.data(), b_vals.size());

//     // 3. 使用 osqp-eigen 进行优化
//     OsqpEigen::Solver solver;
//     std::cout<<  "OSQP solver initialized"<<  std::endl;

//     solver.settings()->setWarmStart(true);
//     solver.settings()->setVerbosity(false);
//     solver.settings()->setMaxIteration(200);
//     solver.settings()->setAbsoluteTolerance(1e-3);
//     solver.settings()->setRelativeTolerance(1e-3);
//     std::cout<<  "OSQP solver started"<<  std::endl;

//     solver.data()->setNumberOfVariables(dim * N);
//     solver.data()->setNumberOfConstraints(row);

//     if (!solver.data()->setHessianMatrix(H))
//         throw std::runtime_error("Failed to set Hessian");
//     if (!solver.data()->setGradient(f))
//         throw std::runtime_error("Failed to set gradient");
//     if (!solver.data()->setLinearConstraintsMatrix(A))
//         throw std::runtime_error("Failed to set constraint matrix");
//         Eigen::VectorXd lower_bound = Eigen::VectorXd::Constant(row, -OsqpEigen::INFTY);
//         if (!solver.data()->setLowerBound(lower_bound))
//             throw std::runtime_error("Failed to set lower bound");
        
//     if (!solver.data()->setUpperBound(b))
//         throw std::runtime_error("Failed to set upper bound");

//     if (!solver.initSolver())
//         throw std::runtime_error("OSQP solver initialization failed");

//     if (!solver.solve())
//         throw std::runtime_error("OSQP failed to solve");

//     Eigen::VectorXd X_opt = solver.getSolution();

//     // 4. 将结果重组成控制点矩阵
//     Eigen::MatrixXd ctrl_opt(dim, N);
//     for (int i = 0; i < N; ++i) {
//         for (int j = 0; j < dim; ++j) {
//             ctrl_opt(j, i) = X_opt(i * dim + j);
//         }
//     }

//     return ctrl_opt;
// }
// void BsplineOptimizer::buildCost(
//     const Eigen::MatrixXd &ctrl_pts,
//     double ts,
//     Eigen::SparseMatrix<double> &H,
//     Eigen::VectorXd &f)
// {
//     int dim = ctrl_pts.rows();           // e.g., 2 for 2D
//     int N   = ctrl_pts.cols();           // 控制点个数
//     int M   = dim * N;                   // 变量维度

//     // 1. 二次平滑项: ∫‖x''(t)‖^2 dt → 离散为 D^T D
//     //    D_shape: ( (N-2)*dim ) x M
//     Eigen::SparseMatrix<double> D((N-2)*dim, M);
//     std::vector<Eigen::Triplet<double>> d_trip;
//     for (int i = 0; i < N-2; ++i) {
//         for (int d = 0; d < dim; ++d) {
//             // 离散二阶差分: x_{i} - 2 x_{i+1} + x_{i+2}
//             int row = i*dim + d;
//             d_trip.emplace_back(row, (i+0)*dim + d,  1.0);
//             d_trip.emplace_back(row, (i+1)*dim + d, -2.0);
//             d_trip.emplace_back(row, (i+2)*dim + d,  1.0);
//         }
//     }
//    // std::cout<<  "D_shape: "<< D.rows() << " x " << D.cols() << std::endl;
//     D.setFromTriplets(d_trip.begin(), d_trip.end());

//     // Hessian = 2 * D^T * D * (weight / ts)
//     double w_smooth = this->lambda1_;  // 用户配置
//     H = 2.0 * (D.transpose() * D);
//     H *= (w_smooth / ts);

//     // 2. 终点 / 端点拟合项:  w_end * ‖x(T)-x_goal‖^2
//     //    只在最后一个控制点上加二次项
//     double w_end = this->lambda4_;
//     for (int d = 0; d < dim; ++d) {
//         int idx = (N-1)*dim + d;
//         H.coeffRef(idx, idx) += 2.0 * w_end;
//     }

//     // 3. 线性项 f: 如果要加线性偏置（比如目标偏置），这里可设置 f(idx) = -2 w_end * goal[d]
//     f = Eigen::VectorXd::Zero(M);
//     for (int d = 0; d < dim; ++d) {
//         int idx = (N-1)*dim + d;
//         f(idx) = -2.0 * w_end * target_point_(d);
//     }
// }

 }