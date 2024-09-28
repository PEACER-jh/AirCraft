#include "solvepnp_node.hpp"

namespace ac_solver
{
SolvePnPNode::SolvePnPNode(const rclcpp::NodeOptions & options) : rclcpp::Node("solver_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Node [ %s ] is started ", this->get_name());

    this->initSolver();
    this->contour_.resize(4);
    this->arm_offset_x_ = this->declare_parameter<double>("arm_offset_x", 0);
    this->arm_offset_y_ = this->declare_parameter<double>("arm_offset_y", 0);

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

    double factor = 2.0 * sqrt(2.0);
    billiards_.push_back(cv::Point3d(0.0, -BilliardsSize / factor,  BilliardsSize / factor));
    billiards_.push_back(cv::Point3d(0.0, -BilliardsSize / factor, -BilliardsSize / factor));
    billiards_.push_back(cv::Point3d(0.0,  BilliardsSize / factor, -BilliardsSize / factor));
    billiards_.push_back(cv::Point3d(0.0,  BilliardsSize / factor,  BilliardsSize / factor));
}


void SolvePnPNode::ContourCallBack(const geometry_msgs::msg::PolygonStamped::SharedPtr contour)
{
    this->contour_.clear();
    this->contour_.resize(4);
    std::vector<cv::Point> temp;
    for(auto & point : contour->polygon.points)
    {
        cv::Point p = cv::Point(point.x, point.y);
        temp.push_back(p);
    }
    std::sort(temp.begin(), temp.end(), [](cv::Point a, cv::Point b){return a.y < b.y;});
    this->contour_[0] = temp[0].x < temp[1].x ? temp[0] : temp[1];
    this->contour_[1] = temp[0].x < temp[1].x ? temp[1] : temp[0];
    this->contour_[2] = temp[2].x > temp[3].x ? temp[2] : temp[3];
    this->contour_[3] = temp[2].x > temp[3].x ? temp[3] : temp[2];

    cv::Mat tVec, rVec;
    if(contour->header.frame_id == "RubikCube")
        cv::solvePnP(rubikcube_, contour_, camera_matrix, dist_coeffs_, rVec, tVec, false, cv::SOLVEPNP_ITERATIVE);
    if(contour->header.frame_id == "Billiards");
        cv::solvePnP(billiards_, contour_, camera_matrix, dist_coeffs_, rVec, tVec, false, cv::SOLVEPNP_ITERATIVE);

    pose_.header = contour->header;
    pose_.pose.position.x = tVec.at<double>(0);
    pose_.pose.position.y = tVec.at<double>(1);
    pose_.pose.position.z = tVec.at<double>(2);
    tf2::Quaternion q;
    cv::Mat R;
    cv::Rodrigues(rVec, R);
    q.setRPY(atan2(R.at<double>(2, 1), R.at<double>(2, 2)),
             atan2(-R.at<double>(2, 0), sqrt(R.at<double>(2, 1) * R.at<double>(2, 1) + R.at<double>(2, 2) * R.at<double>(2, 2))),
             atan2(R.at<double>(1, 0), R.at<double>(0, 0)));
    pose_.pose.orientation.x = q.x();
    pose_.pose.orientation.y = q.y();
    pose_.pose.orientation.z = q.z();
    pose_.pose.orientation.w = q.w();
    this->pose_pub_->publish(pose_);
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ac_solver::SolvePnPNode)