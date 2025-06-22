#include "PTS_Map/PTS_Map.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <chrono>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

using namespace std;
using namespace message_filters;

PTS_MAP::PTS_MAP(){
    init_params();
}

void PTS_MAP::init_occmaps(){
    origin_pose.position.x = occ_origin_x;
    origin_pose.position.y = - (occ_height/2) * grid_size;
    origin_pose.position.z = occ_origin_z;
    origin_pose.orientation.w = 1.0;

    nav_msgs::MapMetaData map_info;
    std_msgs::Header map_header;
    map_info.resolution = grid_size;
    map_info.width = occ_width;
    map_info.height = occ_height;
    map_info.origin = origin_pose;
    map_header.frame_id = "global_gravity_aligned_map";
    map_header.stamp = curr_stamp;
    trmap.set_map_header_info(map_info, map_header);

    trmap.slope_map.data.resize(map_info.width * map_info.height, 0);
    trmap.height_map.data.resize(map_info.width * map_info.height, 0);
    trmap.constraint_map.data.resize(map_info.width * map_info.height, 0);
    trmap.step_map.data.resize(map_info.width * map_info.height, 0);
    trmap.tranversability_cost_map.data.resize(map_info.width * map_info.height, 0);

    // trmap.offset[0] = static_cast<int>(occ_width / 2);
    // trmap.offset[1] = static_cast<int>(occ_height / 2);
    trmap.offset[0] = static_cast<int>(- origin_pose.position.x / grid_size);
    trmap.offset[1] = static_cast<int>(- origin_pose.position.y / grid_size);
}

void PTS_MAP::pcl_to_grid_cell(const pcl::PointCloud<pcl::PointXYZ> &cloud_in){
    for(int i=0; i<cloud_in.size(); i++){
        int x = static_cast <int>(floor(cloud_in.points[i].x/grid_size));
        int y = static_cast <int>(floor(cloud_in.points[i].y/grid_size));
        currentgrid[{x,y}].pcd.push_back(cloud_in.points[i]);

    }
}

void PTS_MAP::init_params(){
    nh.param("/grid_size", grid_size, 2.0);
    nh.param("/occ_width", occ_width, 500);
    nh.param("/occ_height", occ_height, 500);
    nh.param("/occ_origin_x", occ_origin_x, -10.0);
    nh.param("/occ_origin_z", occ_origin_z, -3.0);
    // nh.param("/occ_origin_z", occ_origin_y, -3.0);
    nh.getParam("/input_pcd_topic", input_pcd_topic);
    nh.getParam("/odometry_topic", odometry_topic);
    nh.getParam("/gravity_topic", gravity_topic);
    nh.getParam("/path_topic", path_topic);

    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>()); //use same value from fastlio
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>()); //use same value from fastlio
    nh.param("crop_z", crop_z, 2.5);
    nh.param("/plane_fitting_iter", plane_fitting_iter, 3);
    nh.param("/min_pts_num", min_pts_num, 10);
    nh.param("/d_th", d_th, 0.3);
    nh.param("/min_range", min_range, 1.0);
    nh.param("/max_range", max_range, 15.0);
    nh.param("/temporal_gamma", temporal_gamma, 0.7);
    nh.param("/uncertainty_thr", ground_uncertainty_thr, 0.1);
    nh.param("/uncertainty_criteria", ground_uncertainty_criteria, 0);
    nh.param("/ground_surface_state_propagation_noise", ground_surface_state_propagation_noise, 1e-3);
    nh.param("/EHR_max_dist", EHR_max_dist, 1.0);
    nh.param("/EHR_min_dist", EHR_min_dist, 1.0);

    nh.getParam("/frame_id", frame_id);
    nh.param("/neighbor_cell", neighbor_cell, 1);

    nh.param("/max_number_maintain", max_number_maintain, 2000.0);
    nh.param("/elevation_thr", elevation_thr, 0.7);
    nh.param("/slope_thr", slope_thr, 40.0);
    nh.param("/step_thr", step_thr, 45.0);
    nh.param("/constraint_thr", constraint_thr, 100.0);
    nh.param("/verbose_time", verbose_time, false);
    nh.param("/visualize_near_normal", visualize_near_normal, true);
    nh.param("/visualize_normal_radius", visualize_normal_radius, 8.0);
    nh.param("/weights_coeff", weights_coeff, std::vector<double>{1.0,1.0,1.0});
    nh.param("/debug_process", debug_process, true);
    
    slope_thr = std::sin(M_PI * slope_thr / 180.0);
    step_thr = std::tan(M_PI * step_thr / 180.0);
    
    occPub = nh.advertise<nav_msgs::OccupancyGrid>("/occmap", 1, true);
    slope_occPub = nh.advertise<nav_msgs::OccupancyGrid>("/slope_occmap", 1, true);
    elevation_occPub = nh.advertise<nav_msgs::OccupancyGrid>("/elevation_occmap", 1, true);
    step_occPub = nh.advertise<nav_msgs::OccupancyGrid>("/step_occmap", 1, true);
    constraint_occPub = nh.advertise<nav_msgs::OccupancyGrid>("/constraint_occmap", 1, true);
    pathPublisher = nh.advertise<nav_msgs::Path>(path_topic, 1, true);
    normal_arrow_publisher = nh.advertise<visualization_msgs::MarkerArray>("/normal_arrow", 1, true);
    initial_normal_arrow_publisher = nh.advertise<visualization_msgs::MarkerArray>("/initial_normal_arrow", 1, true);
    odomPub = nh.advertise<nav_msgs::Odometry>("/odometry", 1, true);
    input_pcd_Pub = nh.advertise<sensor_msgs::PointCloud2>("input_pcd", 1, true);

    L2I << extrinR[0], extrinR[1], extrinR[2], extrinT[0],
        extrinR[3], extrinR[4], extrinR[5], extrinT[1],
        extrinR[6], extrinR[7], extrinR[8], extrinT[2],
        0,0,0,1 ;
    init = true;

}

template<typename Point>
bool point_z_cmp(Point a, Point b) {
    return a.z < b.z;
}

double normalCDF(double x, double mu, double sigma) {
    return 0.5 * (1 + erf((x - mu) / (sigma * sqrt(2))));
}


void PTS_MAP::extract_ground_seeds( // spatially distributed
        vector<pcl::PointCloud<pcl::PointXYZ>> pcd_vector_from_neighbor,
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_seeds_pcd) {
    pcl::PointCloud<pcl::PointXYZ> candidates;

    while(1){
        for(int i=0; i< pcd_vector_from_neighbor.size(); i++){
            if(pcd_vector_from_neighbor[i].points.size()>0){
                candidates.push_back(pcd_vector_from_neighbor[i].points[0]);
                pcd_vector_from_neighbor[i].points.erase(pcd_vector_from_neighbor[i].points.begin());
            }
        }
        if(candidates.points.size() >= min_pts_num) break;
    }

    sort(candidates.points.begin(), candidates.points.end(), point_z_cmp<pcl::PointXYZ>);
    int N_cand = candidates.points.size();
    double mean_z = 0;
    double sq_mean_z = 0;
    double std_z = 0;
    int count=0;
    for(int i=0; i<candidates.points.size(); i++){
        if(i<=N_cand/2){
            count++;
            mean_z += candidates.points[i].z;
            sq_mean_z += pow(candidates.points[i].z,2);
        }
    }
    mean_z /= count;
    sq_mean_z /= count;
    
    std_z = sqrt((sq_mean_z - mean_z*mean_z)*count / (count-1));
    for(int i=0; i<candidates.points.size(); i++){
        if(candidates.points[i].z < mean_z + 2*std_z && candidates.points[i].z > mean_z - 2*std_z){
            ground_seeds_pcd->push_back(candidates.points[i]);
        }
    }
}


Eigen::Matrix4d odom2Eigen(const nav_msgs::Odometry::ConstPtr &odom_msg){
    Eigen::Matrix4d mat;
    double q0 = odom_msg->pose.pose.orientation.w;
    double q1 = odom_msg->pose.pose.orientation.x;
    double q2 = odom_msg->pose.pose.orientation.y;
    double q3 = odom_msg->pose.pose.orientation.z;
    mat(0,3) = odom_msg->pose.pose.position.x;
    mat(1,3) = odom_msg->pose.pose.position.y;
    mat(2,3) = odom_msg->pose.pose.position.z;
    mat(0,0) = 2*(q0*q0 + q1*q1)-1;
    mat(0,1) = 2*(q1*q2 - q0*q3);
    mat(0,2) = 2*(q1*q3 + q0*q2);
    mat(1,0) = 2*(q1*q2 + q0*q3);
    mat(1,1) = 2*(q0*q0 + q2*q2)-1;
    mat(1,2) = 2*(q2*q3 - q0*q1);
    mat(2,0) = 2*(q1*q3 - q0*q2);
    mat(2,1) = 2*(q2*q3 + q0*q1);
    mat(2,2) = 2*(q0*q0 + q3*q3)-1;
    mat(3,0) = 0.0;
    mat(3,1) = 0.0;
    mat(3,2) = 0.0;
    mat(3,3) = 1.0;

    return mat;
}

tf::Quaternion rot2quat(const Eigen::Matrix4d rotation){
    tf::Quaternion q; //x,y,z,w
    double w = sqrt(rotation(0,0) + rotation(1,1) + rotation(2,2) + 1)/2;
    q.setW(w);
    q.setX((rotation(2,1)-rotation(1,2)) / (4 * w));
    q.setY((rotation(0,2)-rotation(2,0)) / (4 * w));
    q.setZ((rotation(1,0)-rotation(0,1)) / (4 * w));

    return q;
}

void PTS_MAP::calc_gravity_align_matrix(Eigen::Matrix4d pose, Eigen::Vector3d gravity){
    // cross product (gravity, (0,0,-1)) = (-gy, gx, 0) -> normalize
    double ux = - gravity[1] / sqrt(pow(gravity[0],2) + pow(gravity[1],2));
    double uy = gravity[0] / sqrt(pow(gravity[0],2) + pow(gravity[1],2));
    double uz = 0;
    // dot product (gravity, (0,0,-1))= (0,0,-gz)
    double theta = acos(-gravity[2] / sqrt(pow(gravity[0],2) + pow(gravity[1],2) + pow(gravity[2],2)));

    Eigen::Matrix4d gravity_align=Eigen::Matrix4d::Identity();

    Eigen::Matrix3d skew;
    skew << 0, -uz, uy,
            uz, 0, -ux,
            -uy, ux, 0;
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity() + std::sin(theta)*skew + (1.0 - std::cos(theta))*skew*skew;
    gravity_align.topLeftCorner(3,3) = rotation;
    // gravity_align = Eigen::Matrix4d::Identity();
    local_gravity_align = gravity_align;

    if(init){
        init = false;
        global_gravity_align = local_gravity_align; // global_gravity_align = frame 1 -> frame 2
        init_occmaps();
        Eigen::Matrix3d rotation = global_gravity_align.topLeftCorner(3,3);
        Eigen::Quaterniond q_eigen(rotation);
        q_eigen.normalize();
        tf::Quaternion q;
        global_gravity_aligned_transform.setOrigin(tf::Vector3(0,0,0));
        q.setW(q_eigen.w());
        q.setX(q_eigen.x());
        q.setY(q_eigen.y());
        q.setZ(q_eigen.z());
        global_gravity_aligned_transform.setRotation(q);
    }

    br.sendTransform( tf::StampedTransform( global_gravity_aligned_transform, curr_stamp, "global_gravity_aligned_map", "map") );
    

}

void PTS_MAP::plane_fitting(const pcl::PointCloud<pcl::PointXYZ>::Ptr ground) {
    
    pcl::computeMeanAndCovarianceMatrix(*ground, plane_fitting_cov, plane_fitting_pc_mean);
    // Singular Value Decomposition: SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(plane_fitting_cov, Eigen::DecompositionOptions::ComputeFullU);
    singular_values = svd.singularValues();
    
    plane_fitting_normal = (svd.matrixU().col(2)); 
    if(svd.matrixU().determinant()<0){
        normal_matrix = - svd.matrixU();
    }
    else normal_matrix = svd.matrixU();
    
    if(plane_fitting_normal(2)<0){
        plane_fitting_normal = -plane_fitting_normal;
    }
    Eigen::Vector3d seeds_mean = plane_fitting_pc_mean.head<3>();
    d_th_ = d_th + (plane_fitting_normal.transpose() * seeds_mean)(0, 0);

}

void PTS_MAP::iterative_normal_estimation(
    pcl::PointCloud<pcl::PointXYZ> pcd_from_neighbor,
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_seeds_pcd) {
    for (int i = 0; i < plane_fitting_iter; i++) {
        plane_fitting(ground_seeds_pcd);
        ground_seeds_pcd->clear();
        
        //pointcloud to matrix
        Eigen::MatrixXd points = pcd_from_neighbor.getMatrixXfMap().topRows(3).transpose().cast<double>();
        Eigen::VectorXd result = points * plane_fitting_normal;
        if (i < plane_fitting_iter-1){
            double mean_r = 0;
            double std_r = 0;
            int count = 0;
            double sq_mean_r = 0;
            for (int r = 0; r < result.rows(); r++) {
                if (result[r] < d_th_) {
                    mean_r += result[r];
                    count++;
                    sq_mean_r += result[r] * result[r];
                }
            }
            
            mean_r /= count;   
            sq_mean_r /= count;
            
            std_r = sqrt((sq_mean_r - mean_r*mean_r)* count / (count-1.0));
            
            for (int r = 0; r < result.rows(); r++) {
                if (result[r] > mean_r - 1.96*std_r && result[r] < mean_r + 1.96*std_r) {
                    ground_seeds_pcd->points.push_back(pcd_from_neighbor[r]);
                }
            }
        }
        else{ 
        }
    } 
}

void PTS_MAP::estimate_normal(Eigen::Matrix4d global_gravity_to_local_gravity) {
    std::chrono::system_clock::time_point t0 = std::chrono::system_clock::now(); 
    std::chrono::duration<double> time_preprocess(0.0);
    std::chrono::duration<double> time_ground_seed(0.0);
    std::chrono::duration<double> time_iterative_pca(0.0);
    for(auto it=currentgrid.grid.begin(); it != currentgrid.grid.end(); it++){
        std::chrono::system_clock::time_point t_start = std::chrono::system_clock::now(); 
        if(it->second.pcd.points.size() == 0) continue;
        int cx = it->first.first;
        int cy = it->first.second;
        pcl::PointCloud<pcl::PointXYZ> pcd_from_neighbor;

        pcl::PointCloud<pcl::PointXYZ> candidates;

        
        for(int i=-neighbor_cell; i<= neighbor_cell; i++){
            for(int j=-neighbor_cell; j<= neighbor_cell; j++){
                if(currentgrid.exists(cx+i, cy+j)){
                    pcl::PointCloud<pcl::PointXYZ> neighbor_cloud = currentgrid.grid[{cx+i, cy+j}].pcd;
                    if(neighbor_cloud.points.size() > 0){
                        candidates.push_back(neighbor_cloud.points[0]);
                        pcd_from_neighbor += neighbor_cloud;
                    }
                }
            }
        }
        if(pcd_from_neighbor.points.size() < min_pts_num) continue;
        
        // if(pcd_from_neighbor.points.size() < min_pts_num) continue;
        std::chrono::system_clock::time_point t_preprocess = std::chrono::system_clock::now(); 
        time_preprocess += t_preprocess - t_start;

        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_seeds_pcd(new pcl::PointCloud<pcl::PointXYZ>);
        // extract_ground_seeds(pcd_vector_from_neighbor, ground_seeds_pcd);

        sort(candidates.points.begin(), candidates.points.end(), point_z_cmp<pcl::PointXYZ>);
        int N_cand = candidates.points.size();
        double mean_z = 0;
        double sq_mean_z = 0;
        double std_z = 0;
        int count=0;
        for(int i=0; i<candidates.points.size(); i++){
            if(i<=N_cand/2){
                count++;
                mean_z += candidates.points[i].z;
                sq_mean_z += pow(candidates.points[i].z,2);
            }
        }
        mean_z /= count;
        sq_mean_z /= count;
        
        std_z = sqrt((sq_mean_z - mean_z*mean_z)*count / (count-1));
        for(int i=0; i<candidates.points.size(); i++){
            if(candidates.points[i].z < mean_z + 1.96*std_z && candidates.points[i].z > mean_z - 1.96*std_z){
                ground_seeds_pcd->push_back(candidates.points[i]);
            }
        }
        std::chrono::system_clock::time_point t_ground_seed = std::chrono::system_clock::now(); 
        time_ground_seed += t_ground_seed - t_preprocess;
        
        iterative_normal_estimation(pcd_from_neighbor, ground_seeds_pcd);
        std::chrono::system_clock::time_point t_normal = std::chrono::system_clock::now(); 
        time_iterative_pca += t_normal - t_ground_seed;
        // convert to global gravity aligned frame
        plane_fitting_normal = global_gravity_to_local_gravity.topLeftCorner(3,3).inverse() * plane_fitting_normal;
        plane_fitting_pc_mean = global_gravity_to_local_gravity.inverse() * plane_fitting_pc_mean;
        
        // convert to centroid position, save into the grid
        it->second.normal = plane_fitting_normal.cast<double>();
        it->second.status = 0;
        double cell_centroid_x = (cx + 0.5) * grid_size;
        double cell_centroid_y = (cy + 0.5) * grid_size;
        double cell_centroid_z = plane_fitting_pc_mean(2) - (plane_fitting_normal(0)*(cell_centroid_x-plane_fitting_pc_mean(0)) + plane_fitting_normal(1)*(cell_centroid_y-plane_fitting_pc_mean(1))) / (plane_fitting_normal(2));
        Eigen::Vector3d cell_centroid;
        cell_centroid << cell_centroid_x, cell_centroid_y, cell_centroid_z;
        it->second.mean = cell_centroid;
    }
    std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();   
    std::chrono::duration<double> total_ = t1-t0;
    n_normal += 1.0;
    t_normal += total_.count();
    if(verbose_time) {
        cout << "[time] estimate_normal : " << total_.count() << "s, average: " << t_normal / n_normal << endl;
        cout << "  - [time] preprocess : " << time_preprocess.count() << " sec..." << endl;
        cout << "  - [time] extract_ground_seed : " << time_ground_seed.count() << " sec..." << endl;
        cout << "  - [time] iterative_pca : " << time_iterative_pca.count() << " sec..." << endl;
    }
}

void PTS_MAP::visualize_initial_normal_arrows() {
    std::chrono::system_clock::time_point t0 = std::chrono::system_clock::now(); 
    visualization_msgs::MarkerArray normal_arrows;
    int id = 0;
    for(auto it=currentgrid.grid.begin(); it != currentgrid.grid.end(); it++){
        if(it->second.status < 0) continue;
        geometry_msgs::Point start_p, end_p;
        start_p.x = it->second.mean(0);
        start_p.y = it->second.mean(1);
        start_p.z = it->second.mean(2);
        end_p.x = start_p.x + it->second.normal(0)*0.45;
        end_p.y = start_p.y + it->second.normal(1)*0.45;
        end_p.z = start_p.z + it->second.normal(2)*0.45;
        visualization_msgs::Marker normal_arrow;
        normal_arrow.header.frame_id = "global_gravity_aligned_map";
        normal_arrow.type = visualization_msgs::Marker::ARROW;
        normal_arrow.color.a = 1.0;

        normal_arrow.color.g = 1.0f;
        normal_arrow.pose.orientation.w = 1.0;
        normal_arrow.id = id;
        normal_arrow.scale.x = 0.08; // 0.04 shaft diameter
        normal_arrow.scale.y = 0.17; // 0.09 head diameter
        normal_arrow.scale.z = 0.15; // 0.13 head length
        normal_arrow.points.push_back(start_p);
        normal_arrow.points.push_back(end_p);
        normal_arrows.markers.push_back(normal_arrow);
        id++;

    }
    initial_normal_arrow_publisher.publish(normal_arrows);
    std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();   
    std::chrono::duration<double> total_ = t1-t0;
    if(verbose_time) cout << "[time] visualize_initial_normal_arrows : " << total_.count() << " sec..." << endl;
}

void PTS_MAP::visualize_normal_arrows() {
    std::chrono::system_clock::time_point t0 = std::chrono::system_clock::now();   
    visualization_msgs::MarkerArray normal_arrows;
    int id = 0;
    for(auto it=globalgrid.grid.begin(); it != globalgrid.grid.end(); it++){
        if(it->second.valid_ground != 1 || it->second.valid_elevation != 1) continue;
        int cx = it->first.first;
        int cy = it->first.second;

        Eigen::Matrix4d pose_on_grid = global_gravity_align*pose;
        if(visualize_near_normal){
            int cx_robot = static_cast<int>(pose_on_grid(0,3) / grid_size);
            int cy_robot = static_cast<int>(pose_on_grid(1,3) / grid_size);
            double r = (std::pow((cx_robot - cx),2) + std::pow((cy_robot - cy), 2)) * std::pow(grid_size, 2);
            if(r > std::pow(visualize_normal_radius, 2)) continue;
        }
        
        geometry_msgs::Point start_p, end_p;
        start_p.x = it->second.mean(0);
        start_p.y = it->second.mean(1);
        start_p.z = it->second.mean(2);
        double arrow_length = ((it->second.elevation_mean) + 0.1)*1.2;
        // double arrow_length = ((it->second.elevation_mean) + 2.0*(it->second.elevation_var) + 0.1)*1.2;
        
        end_p.x = start_p.x + it->second.normal(0)*arrow_length;
        end_p.y = start_p.y + it->second.normal(1)*arrow_length;
        end_p.z = start_p.z + it->second.normal(2)*arrow_length;
        visualization_msgs::Marker normal_arrow;
        normal_arrow.header.frame_id = "global_gravity_aligned_map";
        normal_arrow.type = visualization_msgs::Marker::ARROW;
        normal_arrow.color.a = 1.0;
        
        int index = (cy+trmap.offset[1]) * trmap.info.width + (cx+trmap.offset[0]);
        double color_ratio = std::max(0.0, std::min(1.0, trmap.tranversability_cost_map.data[index] / 100.0));
        // if(color_ratio > 0.8) color_ratio = 1.0;
        if(trmap.constraint_map.data[index] == 100) color_ratio = 1.0;
        normal_arrow.color.r = color_ratio;
        normal_arrow.color.g = 1.0f - color_ratio;
        
        normal_arrow.pose.orientation.w = 1.0;
        normal_arrow.id = id;
        normal_arrow.scale.x = 0.1; // shaft diameter
        normal_arrow.scale.y = 0.15; // head diameter
        normal_arrow.scale.z = 0.20; // head length
        normal_arrow.points.push_back(start_p);
        normal_arrow.points.push_back(end_p);
        normal_arrows.markers.push_back(normal_arrow);
        id++;

    }
    normal_arrow_publisher.publish(normal_arrows);
    std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();   
    std::chrono::duration<double> total_ = t1-t0;
    if(verbose_time) cout << "[time] visualize_normal_arrows : " << total_.count() << " sec..." << endl;
}


void PTS_MAP::visualize_path_on_gridmap(Eigen::Matrix4d pose) {
    static int jj =0;
    if(jj%5==0){
        Eigen::Matrix4d pose_on_grid = global_gravity_align*pose;
        geometry_msgs::PoseStamped temp;
        temp.pose.position.x = pose_on_grid(0,3);
        temp.pose.position.y = pose_on_grid(1,3);
        temp.pose.position.z = trmap.slope_map.info.origin.position.z;
        temp.pose.orientation.w = 1;
        temp.header.frame_id = "global_gravity_aligned_map";
        temp.header.stamp = curr_stamp;
        path_on_grid.poses.push_back(temp);
        path_on_grid.header.stamp = curr_stamp;
        path_on_grid.header.frame_id = "global_gravity_aligned_map";
        pathPublisher.publish(path_on_grid);
    }
    jj++;
}

void PTS_MAP::above_ground_elevation_state_update() {
    std::chrono::system_clock::time_point t0 = std::chrono::system_clock::now();   
    // for(auto it = currentgrid.grid.begin(); it != currentgrid.grid.end(); it++){
    //     int cx = it->first.first;
    //     int cy = it->first.second;
    for (auto it=update_cell.begin(); it !=update_cell.end(); it++){
        int cx = it->first;
        int cy = it->second;
        // if(!globalgrid.exists(cx,cy)) continue;
        // if(globalgrid[{cx,cy}].valid_ground != 1) continue; // not update
        if(currentgrid[{cx,cy}].status != 2) continue;
        // if(currentgrid[{cx,cy}].status != -1) continue;
        
        pcl::PointCloud<pcl::PointXYZ> cur_grid_cloud = currentgrid[{cx,cy}].pcd; // pcd here is with respect to local gravity aligned frame;
        pcl::transformPointCloud(cur_grid_cloud, cur_grid_cloud, global_gravity_to_local_gravity.inverse().cast<float>());
        
        Eigen::MatrixXd points = cur_grid_cloud.getMatrixXfMap().topRows(3).transpose().cast<double>();
        Eigen::Vector3d centroid = globalgrid[{cx,cy}].mean;
        Eigen::Vector3d normal = globalgrid[{cx,cy}].normal;

        int n = 0;
        double mean_z = 0.0;
        double sq_mean_z = 0.0;

        for(int i=0; i< cur_grid_cloud.points.size(); i++){
            // double dist_from_ground = std::max(cur_grid_cloud.points[i].z - centroid(2), 0.0);
            double dist_from_ground =cur_grid_cloud.points[i].z - centroid(2);
            if(EHR_min_dist <dist_from_ground && dist_from_ground < EHR_max_dist){
                mean_z += dist_from_ground;
                sq_mean_z += std::pow(dist_from_ground, 2);
                n++;
            }
        }
        if(n <= 1) continue;  // skip when number of points are equal or less than 1
        double curr_n = static_cast<double>(n);
        mean_z /= curr_n;
        sq_mean_z /= curr_n;
        double var = (sq_mean_z - std::pow(mean_z, 2)) * curr_n / (curr_n - 1.0);

        double prev_centroid_z = globalgrid[{cx,cy}].prev_mean(2);

        double prev_mean = globalgrid[{cx,cy}].elevation_mean + (prev_centroid_z - centroid(2));
        double prev_var = globalgrid[{cx,cy}].elevation_var;
        double prev_n = globalgrid[{cx,cy}].elevation_pseudo_count;
        
        double updated_mean, updated_var, updated_num;
        // translate previous mean 
        double t = (EHR_max_dist - prev_mean) / std::sqrt(prev_var);
        double t2 = (EHR_min_dist -prev_mean) / std::sqrt(prev_var);
        double phi = 1.0 / std::sqrt(2.0 * M_PI) * std::exp(-0.5 * t * t);
        double phi2 = 1.0 / std::sqrt(2.0 * M_PI) * std::exp(-0.5 * t2 * t2);
        double PHI = normalCDF(t, 0.0, 1.0);
        double PHI2 = normalCDF(t2, 0.0, 1.0);
        double truncated_mean = prev_mean - std::sqrt(prev_var) * (phi - phi2) / (PHI - PHI2);
        double truncated_var = prev_var * (1.0 - (t2 * phi2 - t * phi)/(PHI2 - PHI) - std::pow(((phi - phi2) / (PHI - PHI2)), 2));
        double truncated_n = std::floor(prev_n * (PHI-PHI2) * temporal_gamma);

        if(truncated_n < 1){
            updated_mean = mean_z;
            updated_var = var;
            updated_num = n;
        }
        else{
            updated_mean = (curr_n * mean_z + truncated_n * truncated_mean) / (curr_n + truncated_n);
            updated_var = ((curr_n-1.0) * var + (truncated_n - 1.0) * truncated_var + curr_n * truncated_n / (curr_n + truncated_n) * std::pow(mean_z - truncated_mean, 2)) / (curr_n + truncated_n - 1.0);
            updated_num = std::min(max_number_maintain, truncated_n + curr_n);
            
        }
        globalgrid[{cx,cy}].elevation_mean = updated_mean;
        globalgrid[{cx,cy}].elevation_var = updated_var;
        globalgrid[{cx,cy}].elevation_pseudo_count = updated_num;
        globalgrid[{cx,cy}].valid_elevation = 1;
        
    }
    std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();   
    std::chrono::duration<double> total_ = t1-t0;
    t_above_ground += total_.count();
    n_above_ground += 1.0;
    if(verbose_time) cout << "[time] above_ground_elevation_state_update : " << total_.count() << "s, average: " << t_above_ground / n_above_ground << std::endl;
}

void PTS_MAP::compute_ground_step(){
    std::chrono::system_clock::time_point t0 = std::chrono::system_clock::now();   

    for (auto it=update_cell.begin(); it !=update_cell.end(); it++){
        int cx = it->first;
        int cy = it->second;

        GlobalGridCell cell = globalgrid[{cx,cy}];
        double step = 0.0;
        for(int i=-1; i<=1; i++){
            for(int j=-1; j<=1; j++){
                if(i==0 && j==0) continue;
                if(!globalgrid.exists(cx+i, cy+j)) continue;
                if(globalgrid[{cx+i,cy+j}].valid_ground != 1) continue;
                double z_diff = std::abs(cell.mean(2) - globalgrid[{cx+i,cy+j}].mean(2));
                Eigen::Vector3d c_n = globalgrid[{cx+i,cy+j}].mean;
                double dist = grid_size * std::sqrt(static_cast<double>(i*i + j*j));
                step = std::max(step, z_diff / dist);
            }
        }
        globalgrid[{cx,cy}].h_grad = step;
    }
    
    std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();   
    std::chrono::duration<double> total_ = t1-t0;
    t_step += total_.count();
    n_step += 1.0;
    if(verbose_time) cout << "[time] compute_ground_step : " << total_.count() << "s, average: " << t_step / n_step << endl;
}

void PTS_MAP::update_traversability_map() {
    std::chrono::system_clock::time_point t0 = std::chrono::system_clock::now();  

    for (auto it=update_cell.begin(); it !=update_cell.end(); it++){
        int cx = it->first;
        int cy = it->second; 
    
        // if(!globalgrid.exists(cx,cy)) continue;
        if(globalgrid[{cx,cy}].valid_elevation != 1) continue; // not update
        int H = trmap.info.height;
        int W = trmap.info.width;
        int index = (cy+trmap.offset[1]) * W + (cx+trmap.offset[0]);
        if(cy+trmap.offset[1] >= H || cy+trmap.offset[1] < 0 || cx+trmap.offset[0] >=W || cx+trmap.offset[0] < 0) continue;

        Eigen::Matrix4d pose_on_grid = global_gravity_align*pose;
        int cx_robot = static_cast<int>(pose_on_grid(0,3) / grid_size);
        int cy_robot = static_cast<int>(pose_on_grid(1,3) / grid_size);
        double r = (std::pow((cx_robot - cx),2) + std::pow((cy_robot - cy), 2)) * std::pow(grid_size, 2);
        bool add_constraint = false;
        if(!globalgrid.exists(cx, cy) || (globalgrid[{cx,cy}].valid_ground != 1 || globalgrid[{cx,cy}].valid_elevation != 1)){
            if(r < 9 && r> min_range*min_range){
                add_constraint = true;
            }
        }
        else{
            GlobalGridCell cell = globalgrid[{cx,cy}];
            
            // slope cost
            double slope = std::acos(cell.normal(2));
            double slope_score = std::max(std::min((std::sin(slope) / slope_thr), 1.0), 0.0) * 100;
            trmap.slope_map.data[index] = static_cast<int>(slope_score);

            // elevation cost
            double elevation = cell.elevation_mean + 2 * std::sqrt(cell.elevation_var);
            double elevation_score = std::max(std::min((elevation / elevation_thr), 1.0), 0.0) * 100;
            trmap.height_map.data[index] = static_cast<int>(elevation_score);

            //step cost
            double step = cell.h_grad;
            double step_score = std::max(std::min(step / step_thr, 1.0), 0.0) * 100;
            trmap.step_map.data[index] = static_cast<int>(step_score);
            
            //total cost
            double weights_sum = 0.0;
            for(int k=0; k< weights_coeff.size(); k++) weights_sum += weights_coeff[k];
            double slope_weight = weights_coeff[0] / weights_sum;
            double height_weight = weights_coeff[1] / weights_sum;
            double step_weight = weights_coeff[2] / weights_sum;

            trmap.tranversability_cost_map.data[index] = static_cast<int>(slope_score * slope_weight + elevation_score * height_weight + step_score * step_weight);
            if(trmap.tranversability_cost_map.data[index] >=99) trmap.tranversability_cost_map.data[index] = 98;

            if(elevation_score > constraint_thr || slope_score > constraint_thr || step_score > constraint_thr){
                add_constraint = true;
            }
        }
    

        if(add_constraint){
            for(int i=-1; i<=1; i++){
                for(int j=-1; j<=1; j++){
                    int inflation_index = (cy+j+trmap.offset[1]) * W + (cx+i+trmap.offset[0]);
                    trmap.constraint_map.data[inflation_index] = 100;
                }
            }
        }
        else{
            trmap.constraint_map.data[index] = 0;
        }
        // }
    }
    std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();   
    std::chrono::duration<double> total_ = t1-t0;
    n_update += 1.0;
    t_update += total_.count();
    if(verbose_time) cout << "[time] update_traversability_map : " << total_.count() << "s, average: " << t_update / n_update << endl;
   
}


void PTS_MAP::uncertainty_estimation() {
    std::chrono::system_clock::time_point t0 = std::chrono::system_clock::now();   
    for(auto it=currentgrid.grid.begin(); it != currentgrid.grid.end(); it++){
        if(it->second.status == -1) continue;
        int cx = it->first.first;
        int cy = it->first.second;

        std::vector<double> neighbor_variances;
        std::vector<Eigen::Vector3d> neighbor_centroids;
        std::vector<Eigen::Vector3d> neighbor_normals;
        std::vector<std::vector<int>> neighbor_index;

        double mean_z = 0;
        Eigen::Vector3d mean_n = Eigen::Vector3d::Zero();

        std::vector<pcl::PointXYZ> neighbor_points;
        for(int i=-1; i<=1; i++){
            for(int j=-1; j<=1; j++){
                if(i==0 && j==0) continue;
                if(currentgrid.exists(cx+i, cy+j)){ // else if there are neighbors in current grid
                    if(currentgrid[{cx+i, cy+j}].status != -1){
                        neighbor_centroids.push_back(currentgrid[{cx+i,cy+j}].mean);
                        neighbor_normals.push_back(currentgrid[{cx+i,cy+j}].normal);
                        neighbor_index.push_back(std::vector<int>{i,j});
                    }
                }
            }
        }
        if(neighbor_centroids.size() < 4) {
            it->second.status = 0; // outlier
            it->second.status_before_reject = 0;
            continue;
        }
        
        Eigen::Vector2d c{it->second.mean(0), it->second.mean(1)};

        pcl::PointCloud<pcl::PointXYZ> pcd_8;

        // Add each observation to the problem
        for (int i=0; i< neighbor_centroids.size(); i++) {
            pcl::PointXYZ temp_p;
            temp_p.x = (c(0) + neighbor_centroids[i](0))/2.0;
            temp_p.y = (c(1) + neighbor_centroids[i](1))/2.0;
            temp_p.z = neighbor_centroids[i](2) - (neighbor_normals[i](0)*(temp_p.x - neighbor_centroids[i](0)) + neighbor_normals[i](1)*(temp_p.y - neighbor_centroids[i](1))) / neighbor_normals[i](2);
            pcd_8.push_back(temp_p);
        }
        
        Eigen::Vector3d temp_normal = currentgrid[{cx,cy}].normal;
        Eigen::Vector3d temp_centroid = currentgrid[{cx,cy}].mean;
        Eigen::Vector3d temp_state {-temp_normal(0)/temp_normal(2), 
                                    -temp_normal(1)/temp_normal(2),
                                    temp_normal.dot(temp_centroid)/temp_normal(2)};
        Eigen::Matrix3d temp_cov = Eigen::Matrix3d::Zero();
        double count = 0.0;
        for(int i=0; i<pcd_8.size(); i++){
            for(int j=i+1; j<pcd_8.size(); j++){
                for(int k=j+1; k<pcd_8.size(); k++){
                    // check if on same edge
                    if(neighbor_index[i][0] == neighbor_index[j][0] && neighbor_index[j][0] == neighbor_index[k][0]) continue;
                    if(neighbor_index[i][1] == neighbor_index[j][1] && neighbor_index[j][1] == neighbor_index[k][1]) continue;
                    pcl::PointXYZ p1 = pcd_8.points[i];
                    pcl::PointXYZ p2 = pcd_8.points[j];
                    pcl::PointXYZ p3 = pcd_8.points[k];
                    Eigen::Matrix3d A;
                    Eigen::Vector3d b;
                    A << p1.x, p1.y, 1,
                         p2.x, p2.y, 1,
                         p3.x, p3.y, 1;
                    b << p1.z, p2.z, p3.z;
                    if((A.transpose()*A).determinant() !=0){
                        Eigen::Vector3d coeff = (A.transpose() * A).inverse() * A.transpose() * b;
                        Eigen::Vector3d dist = coeff - temp_state;
                        temp_cov += dist * dist.transpose();
                        count += 1.0;
                    }
                    
                }
            }
        }
        temp_cov /= (count-1.0);
        it->second.gr_measurement_cov = temp_cov;
        it->second.gr_measurement_mean = temp_state;
        
        double reject_value;
        if(ground_uncertainty_criteria == 0) reject_value = temp_cov(0,0) * temp_cov(1,1) * temp_cov(2,2);
        else if (ground_uncertainty_criteria == 1) reject_value = std::sqrt(std::abs(temp_cov.determinant()));
        if(reject_value < ground_uncertainty_thr){
            it->second.status = 2;
            it->second.status_before_reject = 1;
        }
        else{
            it->second.status = 1;
            it->second.status_before_reject = 0;
        }
    }

    std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();   
    std::chrono::duration<double> total_ = t1-t0;
    n_uncertainty += 1.0;
    t_uncertainty += total_.count();
    if(verbose_time) cout << "[time] uncertainty_estimation : " << total_.count() << "s, average: " << t_uncertainty / n_uncertainty << std::endl;
}

void PTS_MAP::ground_surface_state_update() {
    std::chrono::system_clock::time_point t0 = std::chrono::system_clock::now();   
    update_range = std::vector<std::vector<int>>{{1000000, 1000000}, {-1000000, -1000000}};
    update_cell.clear();
    
    for(auto it=currentgrid.grid.begin(); it != currentgrid.grid.end(); it++){
        if(it->second.status != 2) continue;

        int cx = it->first.first;
        int cy = it->first.second;
        if(!globalgrid.exists(cx, cy)){ // if (cx, cy) is new to global grid, initialize
            globalgrid[{cx,cy}].mean = currentgrid[{cx,cy}].mean;
            globalgrid[{cx,cy}].normal = currentgrid[{cx,cy}].normal;
            globalgrid[{cx,cy}].last_update_t = curr_stamp.toSec();
        }

        std::vector<Eigen::Vector3d> neighbor_state;
        Eigen::Vector3d neighbor_state_mean = currentgrid[{cx,cy}].gr_measurement_mean;
        Eigen::Matrix3d neighbor_covariance = currentgrid[{cx,cy}].gr_measurement_cov;


        Eigen::Vector3d prev_state = globalgrid[{cx,cy}].gr_state;
        Eigen::Matrix3d prev_cov = globalgrid[{cx,cy}].gr_cov;
        double prev_update_t = globalgrid[{cx,cy}].last_update_t;
        

        prev_cov += Eigen::Matrix3d::Identity() * ground_surface_state_propagation_noise * (curr_stamp.toSec() - prev_update_t); // adding perturbation to covariance
        globalgrid[{cx,cy}].last_update_t = curr_stamp.toSec();

        //Kalman filter
        Eigen::Matrix3d K = prev_cov * (neighbor_covariance + prev_cov).inverse();
        Eigen::Vector3d update_state = prev_state + K * (neighbor_state_mean - prev_state);
        Eigen::Matrix3d update_cov = prev_cov - K * prev_cov;

        globalgrid[{cx,cy}].gr_state = update_state;
        globalgrid[{cx,cy}].gr_cov = update_cov;
        globalgrid[{cx,cy}].update_mean_normal();
        globalgrid[{cx,cy}].valid_ground = 1;

        update_cell.push_back({cx,cy});
    }
    std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();   
    std::chrono::duration<double> total_ = t1-t0;
    n_ground_surface += 1.0;
    t_ground_surface += total_.count();
    if(verbose_time) {
        std::cout << "[time] ground_surface_state_update : " << total_.count() << "s, average: " << t_ground_surface / n_ground_surface << std::endl;
    }
}

void PTS_MAP::callbackNode(const sensor_msgs::PointCloud2::ConstPtr &pcd ,const nav_msgs::Odometry::ConstPtr &odom, const geometry_msgs::Vector3Stamped::ConstPtr &grav){
    std::chrono::system_clock::time_point t0 = std::chrono::system_clock::now();   
    currentgrid.clearGrid();
    current_neighbor_pcd_grid.clear();
    curr_stamp = pcd->header.stamp;
    Eigen::Vector3d imu_gravity; 
    imu_gravity << grav->vector.x, grav->vector.y, grav->vector.z;
    
    pose = odom2Eigen(odom);
    calc_gravity_align_matrix(pose, imu_gravity);  

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_input (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_curr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_local_gravity_aligned (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pcd,*pc_input);


    pcl::transformPointCloud (*pc_input, *pc_local_gravity_aligned, (local_gravity_align*L2I).cast<float>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_input_msg(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i<pc_input->points.size(); i++){
        double temp_range = pow(pc_input->points[i].x, 2)+pow(pc_input->points[i].y, 2)+pow(pc_input->points[i].z, 2);
        pcl::PointXYZ temp_pc;
        temp_pc.x = pc_input->points[i].x;
        temp_pc.y = pc_input->points[i].y;
        temp_pc.z = pc_input->points[i].z;
        if(temp_range > min_range * min_range && temp_range < max_range * max_range ){
            if(pc_local_gravity_aligned->points[i].z < crop_z){
                pc_curr->push_back(temp_pc);
                
            }            
            if(pc_local_gravity_aligned->points[i].z < 3.5 ){
                if(debug_process) pc_input_msg->push_back(temp_pc);
            }
            
        }

    }
    if(debug_process){
        pcl::transformPointCloud (*pc_input_msg, *pc_input_msg, (global_gravity_align*pose*L2I).cast<float>()); // transform pcd to global gravity aligned coordinate
        sensor_msgs::PointCloud2 input_cloud_msg;
        pcl::toROSMsg(*pc_input_msg, input_cloud_msg);
        input_cloud_msg.header.frame_id = "global_gravity_aligned_map";
        input_cloud_msg.header.stamp = curr_stamp;
        input_pcd_Pub.publish(input_cloud_msg);
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_gravity_global_aligned(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud (*pc_curr, *pc_gravity_global_aligned, (global_gravity_align*pose*L2I).cast<float>()); // transform pcd to global gravity aligned coordinate
    

    pcl_to_grid_cell(*pc_gravity_global_aligned);
    global_gravity_to_local_gravity= local_gravity_align * pose.inverse() * global_gravity_align.inverse();
    // broadcast local gravity aligned frame
    if(debug_process){
        Eigen::Matrix4d transform_eigen = pose*local_gravity_align.inverse(); // local gravity aligned seen from "map"
        tf::Transform local_gravity_align_transform;
        tf::Quaternion q = rot2quat(transform_eigen);
        local_gravity_align_transform.setOrigin(tf::Vector3(transform_eigen(0,3),transform_eigen(1,3),transform_eigen(2,3)));
        local_gravity_align_transform.setRotation( q );
        br.sendTransform( tf::StampedTransform(local_gravity_align_transform, curr_stamp, "map", "local_gravity_aligned_map" ) );
    }
    
    
    for(auto it=currentgrid.grid.begin(); it != currentgrid.grid.end(); it++){
        pcl::PointCloud<pcl::PointXYZ> grid_pcd = it->second.pcd;
        pcl::transformPointCloud (grid_pcd, grid_pcd, global_gravity_to_local_gravity.cast<float>());
        sort(grid_pcd.points.begin(), grid_pcd.points.end(), point_z_cmp<pcl::PointXYZ>);
        it->second.pcd = grid_pcd;
    }
    std::chrono::system_clock::time_point t2 = std::chrono::system_clock::now();   
    std::chrono::duration<double> total_ = t2 -t0;   
    n_preprocess += 1.0;
    t_preprocess += total_.count();
    cout << "[time] preprocess : " << total_.count() << "s, average: " << t_preprocess / n_preprocess << std::endl;
    estimate_normal(global_gravity_to_local_gravity);
    uncertainty_estimation();
    if(debug_process) visualize_initial_normal_arrows();
    ground_surface_state_update();
    above_ground_elevation_state_update();
    compute_ground_step();
    update_traversability_map();
    if(debug_process) visualize_normal_arrows();

    // publish cost map
    std::chrono::system_clock::time_point t3 = std::chrono::system_clock::now();   
    std_msgs::Header map_header;
    map_header.frame_id = "global_gravity_aligned_map";
    map_header.stamp = curr_stamp;
    trmap.set_map_header(map_header);

    // publish odometry with respect to global gravity aligned frame
    if(debug_process){
        Eigen::Matrix4d pose_gg = global_gravity_align * pose;
        Eigen::Matrix3d rotation_gg = pose_gg.topLeftCorner(3,3);
        Eigen::Quaterniond quaternion_gg(rotation_gg);
        
        nav_msgs::Odometry odomMsg;
        odomMsg.header.stamp = curr_stamp;
        odomMsg.header.frame_id = "global_gravity_aligned_map";  // Set your frame ID appropriately
        odomMsg.child_frame_id = "body";  // Set your child frame ID appropriately
        odomMsg.pose.pose.position.x = pose_gg(0,3);
        odomMsg.pose.pose.position.y = pose_gg(1,3);
        odomMsg.pose.pose.position.z = pose_gg(2,3);
        odomMsg.pose.pose.orientation.w = quaternion_gg.w();
        odomMsg.pose.pose.orientation.x = quaternion_gg.x();
        odomMsg.pose.pose.orientation.y = quaternion_gg.y();
        odomMsg.pose.pose.orientation.z = quaternion_gg.z();
        odomPub.publish(odomMsg);
        
        slope_occPub.publish(trmap.slope_map);
        elevation_occPub.publish(trmap.height_map);
        constraint_occPub.publish(trmap.constraint_map);
        step_occPub.publish(trmap.step_map);
    }
    
    occPub.publish(trmap.tranversability_cost_map);
    
    
    // visualize path on occ grid
    if(debug_process) visualize_path_on_gridmap(pose);

    t2 = std::chrono::system_clock::now();   
    total_ = t2 - t3;
    cout << "[time] final publish : " << total_.count() << " sec..." << endl;
    total_ = t2 -t0;   
    n_total += 1.0;
    t_total += total_.count();
    cout << "[time] estimate_ground : " << total_.count() << "s, average: " << t_total / n_total << std::endl;
    cout << endl;
    
}

void PTS_MAP::run(){
    pc_sub.subscribe(nh, input_pcd_topic, 1);
    pose_sub.subscribe(nh, odometry_topic, 1);
    imu_gravity_sub.subscribe(nh, gravity_topic, 1);

    sync_.reset(new Sync(MySyncPolicy(10),pc_sub, pose_sub, imu_gravity_sub));
    sync_->registerCallback(boost::bind(&PTS_MAP::callbackNode, this, _1, _2, _3));
}

