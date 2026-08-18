#include "transport/ros2/ros_config.hpp"

namespace small_point_lio::transport {

    void read_parameters_from_node(rclcpp::Node &node,
                                   Parameters &parameters,
                                   TransportConfig &transport_config) {
        // 通信层
        transport_config.kind = "ros2";
        transport_config.lidar_topic = node.declare_parameter<std::string>("lidar_topic");
        transport_config.imu_topic = node.declare_parameter<std::string>("imu_topic");
        transport_config.lidar_type = node.declare_parameter<std::string>("lidar_type");
        transport_config.lidar_frame = node.declare_parameter<std::string>("lidar_frame");
        transport_config.save_pcd = node.declare_parameter<bool>("save_pcd");
        // 录制路径是可选的，缺省为空表示不录。
        transport_config.record_path = node.declare_parameter<std::string>("record_path", "");

        // 点云过滤
        parameters.point_filter_num = static_cast<int>(node.declare_parameter<long>("point_filter_num"));
        auto min_distance = node.declare_parameter<double>("min_distance");
        auto max_distance = node.declare_parameter<double>("max_distance");
        parameters.space_downsample = node.declare_parameter<bool>("space_downsample");
        parameters.space_downsample_leaf_size =
                static_cast<float>(node.declare_parameter<double>("space_downsample_leaf_size"));

        // IMU处理
        std::vector<double> gravity_temp = node.declare_parameter<std::vector<double>>("gravity");
        parameters.gravity << gravity_temp[0], gravity_temp[1], gravity_temp[2];
        parameters.check_satu = node.declare_parameter<bool>("check_satu");
        parameters.fix_gravity_direction = node.declare_parameter<bool>("fix_gravity_direction");
        auto raw_satu_acc = node.declare_parameter<double>("satu_acc");
        auto raw_satu_gyro = node.declare_parameter<double>("satu_gyro");
        parameters.acc_norm = node.declare_parameter<double>("acc_norm");

        // 地图
        parameters.map_resolution = node.declare_parameter<double>("map_resolution");
        parameters.init_map_size = static_cast<size_t>(node.declare_parameter<long>("init_map_size"));

        // 雷达与IMU相对位姿
        parameters.extrinsic_est_en = node.declare_parameter<bool>("extrinsic_est_en");
        std::vector<double> extrinsic_T_temp = node.declare_parameter<std::vector<double>>("extrinsic_T");
        parameters.extrinsic_T << extrinsic_T_temp[0], extrinsic_T_temp[1], extrinsic_T_temp[2];
        std::vector<double> extrinsic_R_temp = node.declare_parameter<std::vector<double>>("extrinsic_R");
        parameters.extrinsic_R << extrinsic_R_temp[0], extrinsic_R_temp[1], extrinsic_R_temp[2],
                extrinsic_R_temp[3], extrinsic_R_temp[4], extrinsic_R_temp[5],
                extrinsic_R_temp[6], extrinsic_R_temp[7], extrinsic_R_temp[8];

        // 滤波器参数
        parameters.laser_point_cov = node.declare_parameter<double>("laser_point_cov");
        parameters.imu_meas_acc_cov = node.declare_parameter<double>("imu_meas_acc_cov");
        parameters.imu_meas_omg_cov = node.declare_parameter<double>("imu_meas_omg_cov");
        parameters.velocity_cov = node.declare_parameter<double>("velocity_cov");
        parameters.acceleration_cov = node.declare_parameter<double>("acceleration_cov");
        parameters.omg_cov = node.declare_parameter<double>("omg_cov");
        parameters.ba_cov = node.declare_parameter<double>("ba_cov");
        parameters.bg_cov = node.declare_parameter<double>("bg_cov");
        parameters.plane_threshold = node.declare_parameter<double>("plane_threshold");
        parameters.match_sqaured = node.declare_parameter<double>("match_sqaured");

        // 数据发布
        parameters.publish_odometry_without_downsample =
                node.declare_parameter<bool>("publish_odometry_without_downsample");

        parameters.finalize(min_distance, max_distance, raw_satu_acc, raw_satu_gyro);
    }

}// namespace small_point_lio::transport
