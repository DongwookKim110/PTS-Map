#ifndef G_GPF_H
#define G_GPF_H

#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/format.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/pca.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <cmath>
#include <random>
#include <numeric>
#include <map>
#include <unordered_map>
// #include "utils.hpp"

using Eigen::JacobiSVD;
using namespace std;

class Traversability_Gridmap{
    public:
        nav_msgs::OccupancyGrid slope_map;
        nav_msgs::OccupancyGrid height_map;
        nav_msgs::OccupancyGrid constraint_map;
        nav_msgs::OccupancyGrid step_map;
        nav_msgs::OccupancyGrid tranversability_cost_map;
        nav_msgs::MapMetaData info;
        std_msgs::Header header;

        std::vector<int> offset; // x, y offset of the cost map -> cx_grid = cx + offset[0]
        Traversability_Gridmap(): offset(std::vector<int>{0, 0}){}
        void set_map_header_info(nav_msgs::MapMetaData map_info, std_msgs::Header map_header){
            info = map_info;
            header = map_header;
            slope_map.info = info;
            slope_map.header = header;
            height_map.info = info;
            height_map.header = header;
            constraint_map.info = info;
            constraint_map.header = header;
            step_map.info = info;
            step_map.header = header;
            tranversability_cost_map.info = info;
            tranversability_cost_map.header = header;
        }
        void set_map_header(std_msgs::Header map_header){
            header = map_header;
            slope_map.header = header;
            height_map.header = header;
            constraint_map.header = header;
            step_map.header = header;
            tranversability_cost_map.header = header;
        }

};

struct CurrentGridCell {
    pcl::PointCloud<pcl::PointXYZ> pcd;
    pcl::PointCloud<pcl::PointXYZ> pcd_neighbor;
    pcl::PointCloud<pcl::PointXYZ> pcd_ground_candidates;
    Eigen::Vector3d normal; // ground plane normal
    Eigen::Vector3d mean;  // ground plane center
    int status; // -1: default, 0: normal estimated, 1: outlier, 2: inlier
    int status_before_reject; // 0: outlier, 1: inlier

    Eigen::Vector3d gr_measurement_mean;
    Eigen::Matrix3d gr_measurement_cov;

    CurrentGridCell(): normal(), mean(), status(-1) {}
    CurrentGridCell(const Eigen::Vector3d& n, const Eigen::Vector3d& m, const double& v) : 
        normal(n), mean(m), status(-1) {}
};

struct GlobalGridCell{
    Eigen::Vector3d gr_state; // ground surface state mean
    Eigen::Matrix3d gr_cov;  // ground surface state uncertainty
    Eigen::Vector3d normal; // ground plane normal
    Eigen::Vector3d mean;  // ground plane center
    Eigen::Vector3d prev_mean;  // previous time step ground plane center
    double last_update_t;
    double h_grad;

    int valid_ground; // 1: valid, 0: not valid
    int valid_elevation; // 1: valid, 0: not valid

    double elevation_mean;
    double elevation_var;
    double elevation_pseudo_count;
    
    GlobalGridCell(): gr_state(Eigen::Vector3d::Zero()), gr_cov(Eigen::Matrix3d::Identity()*1e3), normal(), mean(), prev_mean(),
                      elevation_mean(0.0), elevation_var(100000.0), elevation_pseudo_count(0.0), last_update_t(0.0) {}

    void update_mean_normal(){
        prev_mean = mean;
        double x = mean(0);
        double y = mean(1);
        normal(0) = - gr_state(0) / std::sqrt(gr_state(0)*gr_state(0) + gr_state(1)*gr_state(1) + 1.0);
        normal(1) = - gr_state(1) / std::sqrt(gr_state(0)*gr_state(0) + gr_state(1)*gr_state(1) + 1.0);
        normal(2) = 1 / std::sqrt(gr_state(0)*gr_state(0) + gr_state(1)*gr_state(1) + 1.0);
        mean(2) = gr_state(0)*x + gr_state(1)*y + gr_state(2);
    }
};

struct PairHash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);

        // Simple hash combining technique
        return h1 ^ h2;
    }
};

class CurrentGrid{
    public:
        CurrentGrid(): grid() {}
        void insertCell(int x, int y, const CurrentGridCell& cell) {
            grid[std::make_pair(x, y)] = cell;
        }

        // Function to access a cell in the grid
        CurrentGridCell& operator[](const std::pair<int, int>& key) {
            return grid[key];
        }

        // Function to check if a cell exists in the grid
        bool exists(int x, int y) const {
            return grid.find(std::make_pair(x, y)) != grid.end();
        }   
        void clearGrid() {
            grid.clear();
        }
        std::unordered_map<std::pair<int, int>, CurrentGridCell, PairHash> grid;
};

class GlobalGrid{
    public:
        GlobalGrid(): grid() {}
        void insertCell(int x, int y, const GlobalGridCell& cell) {
            grid[std::make_pair(x, y)] = cell;
        }

        // Function to access a cell in the grid
        GlobalGridCell& operator[](const std::pair<int, int>& key) {
            return grid[key];
        }

        // Function to check if a cell exists in the grid
        bool exists(int x, int y) const {
            return grid.find(std::make_pair(x, y)) != grid.end();
        }   
        void clearGrid() {
            grid.clear();
        }
        std::unordered_map<std::pair<int, int>, GlobalGridCell, PairHash> grid;
};

class PTS_MAP{
    public:
        PTS_MAP();
        void run();

    private:
        std::string frame_id ="map";
        Eigen::Matrix4d global_gravity_align;
        Eigen::Matrix4d local_gravity_align;
        nav_msgs::Path path_on_grid;
        
        ros::NodeHandle nh;
        message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
        message_filters::Subscriber<nav_msgs::Odometry> pose_sub;
        message_filters::Subscriber<geometry_msgs::Vector3Stamped> imu_gravity_sub;
        ros::Publisher occPub;
        ros::Publisher elevation_occPub;
        ros::Publisher slope_occPub;
        ros::Publisher constraint_occPub;
        ros::Publisher step_occPub;
        ros::Publisher pathPublisher;
        ros::Publisher normal_arrow_publisher;
        ros::Publisher initial_normal_arrow_publisher;
        ros::Publisher odomPub;
        ros::Publisher input_pcd_Pub;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry, geometry_msgs::Vector3Stamped> MySyncPolicy;  
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;
        int plane_fitting_iter;
        int num_min_pts_;

        int neighbor_cell;

        double ground_uncertainty_thr;
        int ground_uncertainty_criteria;
        double ground_surface_state_propagation_noise;
        double d_th;
        double overhang_dist;
        double temporal_gamma;

        Eigen::VectorXd        singular_values;
        double d_th_;
        Eigen::MatrixXd plane_fitting_normal;
        Eigen::Matrix3d plane_fitting_cov;
        Eigen::Vector4d plane_fitting_pc_mean;
        Eigen::MatrixXd normal_matrix;

        bool init;
        double min_range;
        double max_range;
        double grid_size;
        geometry_msgs::Pose origin_pose;
        ros::Time curr_stamp;
        int occ_height;
        int occ_width;
        double occ_origin_z;
        double occ_origin_x;
        string input_pcd_topic;
        string odometry_topic;
        string gravity_topic;
        string path_topic;

        double crop_z;

        vector<double> extrinT;
        vector<double> extrinR;
        Eigen::Matrix4d L2I;

        tf::Transform global_gravity_aligned_transform;
        Eigen::Matrix4d global_gravity_to_local_gravity;
        double max_number_maintain;
        double elevation_thr;
        double slope_thr;
        double step_thr;
        double EHR_max_dist;
        double EHR_min_dist;
        std::vector<double> weights_coeff;
        double constraint_thr;
        int min_pts_num;

        bool verbose_time;
        bool debug_process;
        bool visualize_near_normal;
        double visualize_normal_radius;
        tf::TransformBroadcaster br;
        
        std::unordered_map<std::pair<int, int>, pcl::PointCloud<pcl::PointXYZ>, PairHash> current_neighbor_pcd_grid;
        std::vector<std::pair<int,int>> update_cell;
        CurrentGrid currentgrid;
        GlobalGrid globalgrid;
        Traversability_Gridmap trmap;
        std::vector<std::vector<int>> update_range; // min, max square range
        std::vector<std::vector<int>> update_index; // (cx,cy) updated index
        Eigen::Matrix4d pose; 

        double n_preprocess, n_normal, n_uncertainty, n_ground_surface, n_above_ground, n_step, n_update, n_total;
        double t_preprocess, t_normal, t_uncertainty, t_ground_surface, t_above_ground, t_step, t_update, t_total;

        void calc_gravity_align_matrix(Eigen::Matrix4d pose, Eigen::Vector3d gravity);

        void plane_fitting(const pcl::PointCloud<pcl::PointXYZ>::Ptr ground);
        void estimate_normal(Eigen::Matrix4d global_gravity_to_local_gravity);
        
        void extract_ground_seeds(
            vector<pcl::PointCloud<pcl::PointXYZ>> pcd_vector_from_neighbor,
            pcl::PointCloud<pcl::PointXYZ>::Ptr ground_pcd);
        void iterative_normal_estimation(
            pcl::PointCloud<pcl::PointXYZ> pcd_from_neighbor,
            pcl::PointCloud<pcl::PointXYZ>::Ptr ground_seeds_pcd);
        
        void init_params();
        void init_occmaps();
        void callbackNode(const sensor_msgs::PointCloud2::ConstPtr &pcd ,const nav_msgs::Odometry::ConstPtr &odom, const geometry_msgs::Vector3Stamped::ConstPtr &grav);
        void pcl_to_grid_cell(const pcl::PointCloud<pcl::PointXYZ> &cloud_in);
        void uncertainty_estimation();
        void uncertainty_estimation_default();
        void visualize_normal_arrows();
        void visualize_initial_normal_arrows();
        void ground_surface_state_update();
        void ground_surface_state_update_default();
        void above_ground_elevation_state_update();
        void compute_ground_step();
        void update_traversability_map();
        void visualize_path_on_gridmap(Eigen::Matrix4d pose);
        void visualize_obstacle_pointcloud();
};

#endif