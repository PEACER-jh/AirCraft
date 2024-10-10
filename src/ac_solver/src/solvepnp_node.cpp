#include "solvepnp_node.hpp"

namespace ac_solver
{
SolvePnPNode::SolvePnPNode(const rclcpp::NodeOptions & options) : rclcpp::Node("solver_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Node [ %s ] is started ", this->get_name());

    this->initSolver();
    this->initKalman();
    this->contour_.resize(4);
    this->arm_offset_x_ = this->declare_parameter<double>("arm_offset_x", 0);
    this->arm_offset_y_ = this->declare_parameter<double>("arm_offset_y", 0);

    this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    this->pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 10);
    this->camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>
                ("/camera/info", rclcpp::SensorDataQoS(), std::bind(&SolvePnPNode::CameraInfoCallBack, this, std::placeholders::_1));
    this->contour_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>
                ("/contour", rclcpp::SensorDataQoS(), std::bind(&SolvePnPNode::ContourCallBack, this, std::placeholders::_1));
}

SolvePnPNode::~SolvePnPNode()
{
    RCLCPP_INFO(this->get_logger(), "Node [ %s ] is stopped ", this->get_name());
}

void SolvePnPNode::CameraInfoCallBack(const sensor_msgs::msg::CameraInfo::ConstSharedPtr info)
{
    RCLCPP_INFO(this->get_logger(), "[ %s ] Receive camera infomation", this->get_name());
    this->cam_info_ = *info;
    this->camera_matrix.create(3, 3, CV_64FC1);
    this->dist_coeffs_.create(1, 5, CV_64FC1);
    for(int i = 0; i < 9; i++)
        this->camera_matrix.at<double>(i / 3, 1 % 3) = this->cam_info_.k[i];
    for(int i = 0; i < 5; i++)
        this->dist_coeffs_.at<double>(0, i) = this->cam_info_.d[i];
    this->camera_info_sub_.reset();
}

void SolvePnPNode::initSolver()
{
    // 边框四点按照从左上角开始顺时针排序
    rubikcube_.push_back(cv::Point3d(0.0, -RubikCubeSize / 2.0,  RubikCubeSize / 2.0));
    rubikcube_.push_back(cv::Point3d(0.0, -RubikCubeSize / 2.0, -RubikCubeSize / 2.0));
    rubikcube_.push_back(cv::Point3d(0.0,  RubikCubeSize / 2.0, -RubikCubeSize / 2.0));
    rubikcube_.push_back(cv::Point3d(0.0,  RubikCubeSize / 2.0,  RubikCubeSize / 2.0));

    // double factor = 2.0 * std::sqrt(2.0);
    // billiards_.push_back(cv::Point3d(0.0, -BilliardsSize / factor,  BilliardsSize / factor));
    // billiards_.push_back(cv::Point3d(0.0, -BilliardsSize / factor, -BilliardsSize / factor));
    // billiards_.push_back(cv::Point3d(0.0,  BilliardsSize / factor, -BilliardsSize / factor));
    // billiards_.push_back(cv::Point3d(0.0,  BilliardsSize / factor,  BilliardsSize / factor));
    billiards_.push_back(cv::Point3d(0.0, -BilliardsSize / 2.0,  BilliardsSize / 2.0));
    billiards_.push_back(cv::Point3d(0.0, -BilliardsSize / 2.0, -BilliardsSize / 2.0));
    billiards_.push_back(cv::Point3d(0.0,  BilliardsSize / 2.0, -BilliardsSize / 2.0));
    billiards_.push_back(cv::Point3d(0.0,  BilliardsSize / 2.0,  BilliardsSize / 2.0));
}

void SolvePnPNode::initKalman()
{
    // 初始化状态转移矩阵 A
    // 假设是匀速模型: 位置和速度
    this->kf_.transitionMatrix = (cv::Mat_<float>(6, 6) << 
        1, 0, 0, 1, 0, 0,  // 位置x和速度x的关系
        0, 1, 0, 0, 1, 0,  // 位置y和速度y的关系
        0, 0, 1, 0, 0, 1,  // 位置z和速度z的关系
        0, 0, 0, 1, 0, 0,  // 速度x保持不变
        0, 0, 0, 0, 1, 0,  // 速度y保持不变
        0, 0, 0, 0, 0, 1   // 速度z保持不变
    );
    // 设置测量矩阵 H
    this->kf_.measurementMatrix = (cv::Mat_<float>(3, 6) <<
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0);
    // 初始化过程噪声协方差矩阵 Q（假设小噪声）
    cv::setIdentity(this->kf_.processNoiseCov, cv::Scalar::all(1e-4));
    // 初始化测量噪声协方差矩阵 R（假设中等噪声）
    cv::setIdentity(this->kf_.measurementNoiseCov, cv::Scalar::all(1e-2));
    // 初始化误差协方差矩阵 P
    cv::setIdentity(this->kf_.errorCovPost, cv::Scalar::all(1));
    // 初始化状态向量（初始位置和速度）
    this->kf_.statePost = (cv::Mat_<float>(6, 1) << 0, 0, 0, 0, 0, 0);


}

void SolvePnPNode::ContourCallBack(const geometry_msgs::msg::PolygonStamped::SharedPtr contour)
{
    // if(contour->polygon.points.size() != 4) return;

    this->contour_.clear();
    this->contour_.resize(4);
    std::vector<cv::Point> temp;
    for(auto & point : contour->polygon.points)
    {
        cv::Point p = cv::Point(point.x, point.y);
        temp.push_back(p);
    }
    std::sort(temp.begin(), temp.end(), [](cv::Point a, cv::Point b){return a.x < b.x;});
    this->contour_[0] = temp[0].y < temp[1].y ? temp[0] : temp[1];
    this->contour_[1] = temp[0].y < temp[1].y ? temp[1] : temp[0];
    this->contour_[2] = temp[2].y > temp[3].y ? temp[2] : temp[3];
    this->contour_[3] = temp[2].y > temp[3].y ? temp[3] : temp[2];

    
    cv::Mat tVec, rVec;
    if(contour->header.frame_id == "RubikCube"){        // 魔方pnp解算
        std::cout << "****** RubikCube ******" << std::endl;
        cv::solvePnP(rubikcube_, contour_, camera_matrix, dist_coeffs_, rVec, tVec, false, cv::SOLVEPNP_ITERATIVE);
    }
    else if(contour->header.frame_id == "Billiards"){   // 台球pnp解算
        std::cout << "****** Billiards ******" << std::endl;
        cv::solvePnP(billiards_, contour_, camera_matrix, dist_coeffs_, rVec, tVec, false, cv::SOLVEPNP_ITERATIVE);
    }
    else{
        std::cout << "****** Unknown Object ******" << std::endl;
        return;
    }

    cv::Mat prediction = this->kf_.predict();
    cv::Mat measurement = (cv::Mat_<float>(3, 1) << tVec.at<double>(0), tVec.at<double>(1), tVec.at<double>(2));
    cv::Mat estimated = this->kf_.correct(measurement);
    cv::Point3f filtered_tvec(estimated.at<float>(0), estimated.at<float>(1), estimated.at<float>(2));

    pose_.header = contour->header;
    // pose_.pose.position.x = tVec.at<double>(0);
    // pose_.pose.position.y = tVec.at<double>(1) + this->arm_offset_x_;
    // pose_.pose.position.z = tVec.at<double>(2) + this->arm_offset_y_;
    this->pose_.pose.position.x = filtered_tvec.x;
    this->pose_.pose.position.y = filtered_tvec.y + this->arm_offset_x_;
    this->pose_.pose.position.z = filtered_tvec.z + this->arm_offset_y_;
    tf2::Quaternion q;
    cv::Mat R;
    cv::Rodrigues(rVec, R);
    q.setRPY(atan2(R.at<double>(2, 1), R.at<double>(2, 2)),
             atan2(-R.at<double>(2, 0), sqrt(R.at<double>(2, 1) * R.at<double>(2, 1) + R.at<double>(2, 2) * R.at<double>(2, 2))),
             atan2(R.at<double>(1, 0), R.at<double>(0, 0)));
    this->pose_.pose.orientation.x = q.x();
    this->pose_.pose.orientation.y = q.y();
    this->pose_.pose.orientation.z = q.z();
    this->pose_.pose.orientation.w = q.w();
    this->pose_pub_->publish(pose_);
    std::cout << "oringin:\t" << pose_.pose.position.x << " " << pose_.pose.position.y << " " << pose_.pose.position.z << std::endl;
    std::cout << "send:\t\t" << -pose_.pose.position.y << " " << -pose_.pose.position.z << std::endl;

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->pose_.header.stamp;
    tf.header.frame_id = "target";
    tf.child_frame_id = "camera";
    tf.transform.translation.x = pose_.pose.position.x * 100;
    tf.transform.translation.y = pose_.pose.position.y * 100;
    tf.transform.translation.z = pose_.pose.position.z * 100;
    tf.transform.rotation.x = pose_.pose.orientation.x;
    tf.transform.rotation.y = pose_.pose.orientation.y;
    tf.transform.rotation.z = pose_.pose.orientation.z;
    tf.transform.rotation.w = pose_.pose.orientation.w;
    this->tf_broadcaster_->sendTransform(tf);

}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ac_solver::SolvePnPNode)