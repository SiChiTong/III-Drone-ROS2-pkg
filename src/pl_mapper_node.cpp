/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "pl_mapper_node.h"

/*****************************************************************************/
// Implementation
/*****************************************************************************/

PowerlineMapperNode::PowerlineMapperNode(const std::string & node_name, const std::string & node_namespace) : 
        rclcpp::Node(node_name, node_namespace),
        powerline_(1., 0.25, this->get_logger(), 0, 60, 150) {
            // first val = r = mmW variance 5
            // second val = q = odo variance 0.005
            // last three values indicate alive_cnt_low_thresh=0, alive_cnt_high_thresh=60, alive_cnt_ceiling=90

    pl_direction_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/pl_dir_computer/powerline_direction", 10, std::bind(&PowerlineMapperNode::plDirectionCallback, this, std::placeholders::_1));

    mmwave_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/iwr6843_pcl", 10, std::bind(&PowerlineMapperNode::mmWaveCallback, this, std::placeholders::_1));

    powerline_pub_ = this->create_publisher<iii_interfaces::msg::Powerline>("powerline", 10);
    points_est_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_est", 10);
    transformed_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_points", 10);
    projected_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("projected_points", 10);

    //individual_pl_pubs_ = new std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>(10); 
    // this->create_publisher<geometry_msgs::msg::PoseStamped>("individual_powerline_poses", 10);
    //projection_plane_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("projection_plane", 10);
    //pl_direction_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("powerline_direction", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    geometry_msgs::msg::TransformStamped mmw_tf;

    while(true) {

        try {

            mmw_tf = tf_buffer_->lookupTransform("drone", "iwr6843_frame", tf2::TimePointZero);

            RCLCPP_INFO(this->get_logger(), "Found mmWave transform, frame drone to iwr6843_frame");
            break;

        } catch(tf2::TransformException & ex) {

            RCLCPP_DEBUG(this->get_logger(), "Could not get mmWave transform, frame drone to iwr6843_frame, trying again...");

        }

    }

	rclcpp::Rate rate(1000ms);
	rate.sleep();

    // Call on_timer function every second
    drone_tf_timer_ = this->create_wall_timer(
      10ms, std::bind(&PowerlineMapperNode::odometryCallback, this));


    quat_t mmw_quat(
        mmw_tf.transform.rotation.w,
        mmw_tf.transform.rotation.x,
        mmw_tf.transform.rotation.y,
        mmw_tf.transform.rotation.z
    );

    R_drone_to_mmw = quatToMat(mmw_quat);

    v_drone_to_mmw(0) = mmw_tf.transform.translation.x;
    v_drone_to_mmw(1) = mmw_tf.transform.translation.y;
    v_drone_to_mmw(2) = mmw_tf.transform.translation.z;

    RCLCPP_INFO(this->get_logger(), "Initialized PowerlineMapperNode");

}

void PowerlineMapperNode::odometryCallback() {

    // RCLCPP_INFO(this->get_logger(), "Fetching odometry transform");

    geometry_msgs::msg::TransformStamped tf;

    try {

        tf = tf_buffer_->lookupTransform("drone", "world", tf2::TimePointZero);

    } catch(tf2::TransformException & ex) {

        RCLCPP_FATAL(this->get_logger(), "Could not get odometry transform, frame drone to world");
        return;

    }

    point_t position(
        tf.transform.translation.x,
        tf.transform.translation.y, 
        tf.transform.translation.z
    );

    quat_t quat(
        tf.transform.rotation.w,
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z
    );

    powerline_.UpdateOdometry(position, quat);

    publishPowerline();

    //publishProjectionPlane();

}

void PowerlineMapperNode::mmWaveCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    // RCLCPP_INFO(this->get_logger(), "Received mmWave message");

    // read PointCloud2 msg data
    int pcl_size = msg->width;
    uint8_t *ptr = msg->data.data();
    const uint32_t POINT_STEP = 12;

    std::vector<point_t> transformed_points;
    std::vector<point_t> projected_points;

    //point_t dummy(0,0,0);

    for (size_t i = 0; i < pcl_size; i++) {

        point_t point(
            *(reinterpret_cast<float*>(ptr + 0)),
            *(reinterpret_cast<float*>(ptr + 4)),
            *(reinterpret_cast<float*>(ptr + 8))
        );

        // filter points based on diagonal distance
        if(sqrt( pow(point(0),2) + pow(point(1),2) + pow(point(2),2) ) < 0.25 ) {
            // RCLCPP_INFO(this->get_logger(), "Point filtered away; below minimum distance");
            continue;
        }

        point = R_drone_to_mmw * point + v_drone_to_mmw;

        point_t projected_point = powerline_.UpdateLine(point);

        ptr += POINT_STEP;

        transformed_points.push_back(point);
        projected_points.push_back(projected_point);

    }   

    // int count = powerline_.GetVisibleLines().size();

    // RCLCPP_INFO(this->get_logger(), "Currently has %d visible lines registered", count);

    // transformed_points.push_back(dummy);
    // projected_points.push_back(dummy);

    publishPoints(transformed_points, transformed_points_pub_);
    publishPoints(projected_points, projected_points_pub_);

    powerline_.CleanupLines();

}

void PowerlineMapperNode::plDirectionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {

    // RCLCPP_INFO(this->get_logger(), "Received powerline direction message");

    quat_t pl_direction;
    pl_direction(0) = msg->pose.orientation.w;
    pl_direction(1) = msg->pose.orientation.x;
    pl_direction(2) = msg->pose.orientation.y;
    pl_direction(3) = msg->pose.orientation.z;

    pl_direction_ = pl_direction; ////////

    powerline_.UpdateDirection(pl_direction);
}

void PowerlineMapperNode::publishPowerline() {

    // RCLCPP_INFO(this->get_logger(), "Publishing powerline");

    std::vector<SingleLine> lines = powerline_.GetVisibleLines();
    //orientation_t plane_orientation = powerline_.GetPlaneOrientation();
    //quat_t plane_quat = eulToQuat(plane_orientation);

    auto msg = iii_interfaces::msg::Powerline();
    auto quat_msg = geometry_msgs::msg::Quaternion();
    auto pcl2_msg = sensor_msgs::msg::PointCloud2();
    pcl2_msg.header.frame_id = "drone";
    pcl2_msg.header.stamp = this->get_clock()->now();

    pcl2_msg.fields.resize(3);
    pcl2_msg.fields[0].name = 'x';
    pcl2_msg.fields[0].offset = 0;
    pcl2_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl2_msg.fields[0].count = 1;
    pcl2_msg.fields[1].name = 'y';
    pcl2_msg.fields[1].offset = 4;
    pcl2_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl2_msg.fields[1].count = 1;
    pcl2_msg.fields[2].name = 'z';
    pcl2_msg.fields[2].offset = 8;
    pcl2_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl2_msg.fields[2].count = 1;

    const uint32_t POINT_STEP = 12;

    if(lines.size() > 0){

        pcl2_msg.data.resize(std::max((size_t)1, lines.size()) * POINT_STEP, 0x00);

    } else {

        // RCLCPP_INFO(this->get_logger(), "No registered powerlines");

    }

    pcl2_msg.point_step = POINT_STEP; // size (bytes) of 1 point (float32 * dimensions (3 when xyz))
    pcl2_msg.row_step = pcl2_msg.data.size();//pcl2_msg.point_step * pcl2_msg.width; // only 1 row because unordered
    pcl2_msg.height = 1; // because unordered cloud
    pcl2_msg.width = pcl2_msg.row_step / POINT_STEP; // number of points in cloud
    pcl2_msg.is_dense = false; // there may be invalid points

    uint8_t *pcl2_ptr = pcl2_msg.data.data();

    // quat_msg.w = plane_quat(0);
    // quat_msg.x = plane_quat(1);
    // quat_msg.y = plane_quat(2);
    // quat_msg.z = plane_quat(3);
    
    quat_msg.w = pl_direction_(0); ////////
    quat_msg.x = pl_direction_(1); ////////
    quat_msg.y = pl_direction_(2); ////////
    quat_msg.z = pl_direction_(3); ////////

    for (int i = 0; i < lines.size(); i++) {
        auto point_msg = geometry_msgs::msg::Point();
        auto pose_msg = geometry_msgs::msg::Pose();
        auto pose_stamped_msg = geometry_msgs::msg::PoseStamped();

        point_t point = lines[i].GetPoint();

        point_msg.x = point(0);
        point_msg.y = point(1);
        point_msg.z = point(2);

        pose_msg.orientation = quat_msg;
        pose_msg.position = point_msg;

        pose_stamped_msg.pose = pose_msg;
        pose_stamped_msg.header.frame_id = "drone";
        pose_stamped_msg.header.stamp = this->get_clock()->now();

        //individual_pl_pubs_->at(i)->publish(pose_stamped_msg);

        msg.poses.push_back(pose_stamped_msg);
        msg.ids.push_back(lines[i].GetId());

        *(reinterpret_cast<float*>(pcl2_ptr + 0)) = point(0);
        *(reinterpret_cast<float*>(pcl2_ptr + 4)) = point(1);
        *(reinterpret_cast<float*>(pcl2_ptr + 8)) = point(2);
        pcl2_ptr += POINT_STEP;

    }

    msg.count = lines.size();

    powerline_pub_->publish(msg);
    points_est_pub_->publish(pcl2_msg);

}

//void PowerlineMapperNode::publishProjectionPlane() {
//
//    RCLCPP_INFO(this->get_logger(), "Publishing projection plane");
//
//    plane_t plane = powerline_.GetProjectionPlane();
//
//    quat_t quat = powerline_.GetDirection();
//
//    auto msg  = geometry_msgs::msg::PoseStamped();
//    auto quat_msg = geometry_msgs::msg::Quaternion();
//    auto point_msg = geometry_msgs::msg::Point();
//
//    quat_msg.w = quat(0);
//    quat_msg.x = quat(1);
//    quat_msg.y = quat(2);
//    quat_msg.z = quat(3);
//
//    point_msg.x = 0;
//    point_msg.y = 0;
//    point_msg.z = 0;
//
//    msg.header.frame_id = "drone";
//    msg.header.stamp = this->get_clock()->now();
//
//    msg.pose.orientation = quat_msg;
//    msg.pose.position = point_msg;
//
//    projection_plane_pub_->publish(msg);
//
//}

void PowerlineMapperNode::publishPoints(std::vector<point_t> points, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub) {

    // RCLCPP_INFO(this->get_logger(), "Publishing points");

    auto pcl2_msg = sensor_msgs::msg::PointCloud2();
    pcl2_msg.header.frame_id = "drone";
    pcl2_msg.header.stamp = this->get_clock()->now();

    pcl2_msg.fields.resize(3);
    pcl2_msg.fields[0].name = 'x';
    pcl2_msg.fields[0].offset = 0;
    pcl2_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl2_msg.fields[0].count = 1;
    pcl2_msg.fields[1].name = 'y';
    pcl2_msg.fields[1].offset = 4;
    pcl2_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl2_msg.fields[1].count = 1;
    pcl2_msg.fields[2].name = 'z';
    pcl2_msg.fields[2].offset = 8;
    pcl2_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl2_msg.fields[2].count = 1;

    const uint32_t POINT_STEP = 12;

    if(points.size() > 0){

        pcl2_msg.data.resize(std::max((size_t)1, points.size()) * POINT_STEP, 0x00);

    } else {

        return;

    }

    pcl2_msg.point_step = POINT_STEP; // size (bytes) of 1 point (float32 * dimensions (3 when xyz))
    pcl2_msg.row_step = pcl2_msg.data.size();//pcl2_msg.point_step * pcl2_msg.width; // only 1 row because unordered
    pcl2_msg.height = 1; // because unordered cloud
    pcl2_msg.width = pcl2_msg.row_step / POINT_STEP; // number of points in cloud
    pcl2_msg.is_dense = false; // there may be invalid points

    uint8_t *pcl2_ptr = pcl2_msg.data.data();

    for (int i = 0; i < points.size(); i++) {
        point_t point = points[i];

        *(reinterpret_cast<float*>(pcl2_ptr + 0)) = point(0);
        *(reinterpret_cast<float*>(pcl2_ptr + 4)) = point(1);
        *(reinterpret_cast<float*>(pcl2_ptr + 8)) = point(2);
        pcl2_ptr += POINT_STEP;

    }

    pub->publish(pcl2_msg);

}

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<PowerlineMapperNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;

}
