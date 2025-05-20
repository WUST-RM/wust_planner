#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <plan_env/edt_environment.hpp>
#include <rclcpp/rclcpp.hpp>

namespace fast_planner
{   class Polyhedron
    {
    public:
        // 超平面表示：n^T * x ≤ d
        struct HalfPlane {
            Eigen::Vector2d normal;
            double offset;  // 即 d
        };
    
        std::vector<HalfPlane> half_planes_;
    
        Polyhedron() {}
    
        /// 添加超平面 n^T * x ≤ d
        void addPlane(const Eigen::Vector2d &n, double d)
        {
            half_planes_.push_back({n.normalized(), d});
        }
    
        /// 判断点是否在 Polyhedron 内部
        bool inside(const Eigen::Vector2d &pt) const
        {
            for (const auto &hp : half_planes_) {
                if (hp.normal.dot(pt) > hp.offset + 1e-6)  // 允许微小误差
                    return false;
            }
            return true;
        }
    
        /// 多面体是否合法（至少三个面）
        bool valid() const
        {
            return half_planes_.size() >= 3;
        }
    
        /// 获取所有平面参数（可用于QP/优化器）
        void getConstraints(Eigen::MatrixXd &A, Eigen::VectorXd &b) const
        {
            size_t n = half_planes_.size();
            A.resize(n, 2);
            b.resize(n);
            for (size_t i = 0; i < n; ++i) {
                A.row(i) = half_planes_[i].normal.transpose();
                b(i) = half_planes_[i].offset;
            }
        }
    };
    class BsplineOptimizer
    {
    public:
        static const int SMOOTHNESS;
        static const int DISTANCE;
        static const int FEASIBILITY;
        static const int ENDPOINT;
        static const int GUIDE;
        static const int WAYPOINTS;

        static const int GUIDE_PHASE;
        static const int NORMAL_PHASE;

        BsplineOptimizer() {
            target_point_ = Eigen::Vector2d::Zero();
        }
        ~BsplineOptimizer() {}
        /* main API */
        void setEnvironment(const EDTEnvironment::Ptr &env);
        void setWaypoints(const vector<Eigen::Vector2d> &waypts,
                          const vector<int> &waypt_idx);
        void setParam(std::shared_ptr<rclcpp::Node> nh);
        Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd &points, const double &ts,
                                            const int &cost_function, int max_num_id, int max_time_id);

        // required inputs
        void setControlPoints(const Eigen::MatrixXd &points);
        void setBsplineInterval(const double &ts);
        void setCostFunction(const int &cost_function);
        void setTerminateCond(const int &max_num_id, const int &max_time_id);
        void optimize();
        bool generateHomotopicCorridor(
            const std::vector<Eigen::Vector2d>& path_pts,
            double safety_radius,
            std::vector<Polyhedron>& corridor);
        // Eigen::MatrixXd optimizeWithCorridor(
        //         const Eigen::MatrixXd& init_ctrl_pts,
        //         double ts,
        //         const std::vector<Polyhedron>& corridor);
        //         void buildCost(
        //             const Eigen::MatrixXd &ctrl_pts,
        //             double ts,
        //             Eigen::SparseMatrix<double> &H,
        //             Eigen::VectorXd &f);
        //             void setTargetPoint(const Eigen::VectorXd &pt) {
        //                 target_point_ = pt;
        //             }

        typedef unique_ptr<BsplineOptimizer> Ptr;

    private:
        /* optimization parameters */
        int order_;      // bspline degree
        double lambda1_; // jerk smoothness weight
        double lambda2_; // distance weight
        double lambda3_; // feasibility weight
        double lambda4_; // end point weight
        double lambda5_; // guide cost weight
        double lambda6_; // visibility cost weight
        double lambda7_; // waypoints cost weight
        double lambda8_; // acc smoothness

        double dist0_;             // safe distance
        double max_vel_, max_acc_; // dynamic limits

        int algorithm1_; // optimization algorithms for quadratic cost
        int algorithm2_; // optimization algorithms for general cost

        int max_iteration_num_[4];     // stopping criteria that can be used
        double max_iteration_time_[4]; // stopping criteria that can be used

        Eigen::MatrixXd control_points_; // B-spline control points, N x dim
        double bspline_interval_;        // B-spline knot span
        Eigen::Vector2d end_pt_;         // end of the trajectory

        vector<Eigen::Vector2d> guide_pts_; // geometric guiding path points, N-6
        vector<Eigen::Vector2d> waypoints_; // waypts constraints
        vector<int> waypt_idx_;             // waypts constraints index
        int max_num_id_, max_time_id_;      // stopping criteria
        int cost_function_;                 // used to determine objective function

        int dim_; // dimension of the B-spline
                  /* intermediate variables */
        /* buffer for gradient of cost function, to avoid repeated allocation and
         * release of memory */
        vector<Eigen::Vector2d> g_q_;
        vector<Eigen::Vector2d> g_smoothness_;
        vector<Eigen::Vector2d> g_distance_;
        vector<Eigen::Vector2d> g_feasibility_;
        vector<Eigen::Vector2d> g_endpoint_;
        vector<Eigen::Vector2d> g_guide_;
        vector<Eigen::Vector2d> g_waypoints_;
        Eigen::VectorXd target_point_;  

        int variable_num_;                  // optimization variables
        int iter_num_;                      // iteration of the solver
        std::vector<double> best_variable_; //
        double min_cost_;                   //
        /* cost function */
        /* calculate each part of cost function with control points q as input */

        static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data);
        void combineCost(const std::vector<double> &x, vector<double> &grad, double &cost);

        // q contains all control points
        void calcSmoothnessCost(const vector<Eigen::Vector2d> &q, double &cost,
                                vector<Eigen::Vector2d> &gradient);
        void calcDistanceCost(const vector<Eigen::Vector2d> &q, double &cost,
                              vector<Eigen::Vector2d> &gradient);
        void calcFeasibilityCost(const vector<Eigen::Vector2d> &q, double &cost,
                                 vector<Eigen::Vector2d> &gradient);
        void calcEndpointCost(const vector<Eigen::Vector2d> &q, double &cost,
                              vector<Eigen::Vector2d> &gradient);
        void calcGuideCost(const vector<Eigen::Vector2d> &q, double &cost, vector<Eigen::Vector2d> &gradient);
        // void calcVisibilityCost(const vector<Eigen::Vector3d> &q, double &cost,
        //                         vector<Eigen::Vector3d> &gradient);
        void calcWaypointsCost(const vector<Eigen::Vector2d> &q, double &cost,
                               vector<Eigen::Vector2d> &gradient);
        // void calcViewCost(const vector<Eigen::Vector3d> &q, double &cost, vector<Eigen::Vector3d> &gradient);
        bool isQuadratic();
        EDTEnvironment::Ptr edt_environment_;
        std::shared_ptr<rclcpp::Node> node_;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
#endif