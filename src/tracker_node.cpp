#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "volleyball_tracker/aero_ekf.hpp"
#include "volleyball_tracker/trajectory_predictor.hpp"

class VolleyballTrackerNode : public rclcpp::Node {
public:
    VolleyballTrackerNode() : Node("volleyball_tracker_node") {
        this->declare_parameter<double>("q_pos", 0.02);
        this->declare_parameter<double>("q_vel", 0.5);
        this->declare_parameter<double>("r_pos", 0.05);
        this->declare_parameter<double>("gate_thresh", 4.0);
        this->declare_parameter<double>("court_x_min", -4.5);
        this->declare_parameter<double>("court_x_max", 4.5);
        this->declare_parameter<double>("court_y_min", 0.0);
        this->declare_parameter<double>("court_y_max", 18.0);
        this->declare_parameter<double>("target_interception_z", 0.8);
        this->declare_parameter<std::string>("frame_id", "camera_stereo_link");

        frame_id_ = this->get_parameter("frame_id").as_string();
        target_z_ = this->get_parameter("target_interception_z").as_double();

        ekf_.setNoiseParams(
            this->get_parameter("q_pos").as_double(),
            this->get_parameter("q_vel").as_double(),
            this->get_parameter("r_pos").as_double());

        predictor_.setCourtDimensions(
            this->get_parameter("court_x_min").as_double(),
            this->get_parameter("court_x_max").as_double(),
            this->get_parameter("court_y_min").as_double(),
            this->get_parameter("court_y_max").as_double());

        raw_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/volleyball/position", rclcpp::SensorDataQoS(),
            std::bind(&VolleyballTrackerNode::positionCallback, this, std::placeholders::_1));

        filtered_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/volleyball/position_filtered", rclcpp::SensorDataQoS());

        landing_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/volleyball/landing_prediction", 10);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/volleyball/trajectory_markers", 10);

        last_time_ = this->now();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), // 100 Hz
            std::bind(&VolleyballTrackerNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "VolleyballTrackerNode initialized.");
    }

private:
    void positionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        rclcpp::Time current_time = msg->header.stamp;
        if (!ekf_.isInitialized()) {
            ekf_.initialize(Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z));
            last_meas_time_ = current_time;
            return;
        }

        double dt = (current_time - last_meas_time_).seconds();
        if (dt > 0.0 && dt < 0.2) {
            ekf_.predict(dt);
        }

        Eigen::Vector3d z(msg->point.x, msg->point.y, msg->point.z);
        double gate = this->get_parameter("gate_thresh").as_double();
        if (ekf_.update(z, gate)) {
            consecutive_lost_ = 0;
            last_meas_time_ = current_time;
            publishFilteredState(current_time);
        }
    }

    void timerCallback() {
        rclcpp::Time now = this->now();
        if (!ekf_.isInitialized()) {
            return;
        }

        double time_since_meas = (now - last_meas_time_).seconds();
        if (time_since_meas > 0.5) {
            // Lost ball tracking after 0.5 seconds
            ekf_.reset();
            return;
        }

        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt > 0.0 && dt < 0.05) {
            ekf_.predict(dt);
        }

        publishFilteredState(now);
    }

    void publishFilteredState(const rclcpp::Time& stamp) {
        Eigen::Vector3d pos = ekf_.getPosition();
        Eigen::Vector3d vel = ekf_.getVelocity();

        // 1. Publish filtered position
        geometry_msgs::msg::PointStamped filt_msg;
        filt_msg.header.stamp = stamp;
        filt_msg.header.frame_id = frame_id_;
        filt_msg.point.x = pos.x();
        filt_msg.point.y = pos.y();
        filt_msg.point.z = pos.z();
        filtered_pub_->publish(filt_msg);

        // 2. Predict landing and publish
        auto pred = predictor_.predictLanding(pos, vel, target_z_);
        if (pred.is_valid) {
            geometry_msgs::msg::PointStamped land_msg;
            land_msg.header.stamp = stamp;
            land_msg.header.frame_id = frame_id_;
            land_msg.point.x = pred.landing_pos.x();
            land_msg.point.y = pred.landing_pos.y();
            land_msg.point.z = pred.landing_pos.z();
            landing_pub_->publish(land_msg);

            publishMarkers(pred, stamp);
        }
    }

    void publishMarkers(const volleyball_tracker::TrajectoryPredictionResult& pred, const rclcpp::Time& stamp) {
        visualization_msgs::msg::MarkerArray markers;

        // Line strip for trajectory
        visualization_msgs::msg::Marker line_strip;
        line_strip.header.frame_id = frame_id_;
        line_strip.header.stamp = stamp;
        line_strip.ns = "trajectory";
        line_strip.id = 0;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.scale.x = 0.03; // line width
        line_strip.color.r = 1.0;
        line_strip.color.g = 0.5;
        line_strip.color.b = 0.0;
        line_strip.color.a = 0.8;

        for (const auto& pt : pred.trajectory_points) {
            geometry_msgs::msg::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z();
            line_strip.points.push_back(p);
        }
        markers.markers.push_back(line_strip);

        // Landing point sphere
        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = frame_id_;
        sphere.header.stamp = stamp;
        sphere.ns = "landing";
        sphere.id = 1;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position.x = pred.landing_pos.x();
        sphere.pose.position.y = pred.landing_pos.y();
        sphere.pose.position.z = pred.landing_pos.z();
        sphere.scale.x = 0.21;
        sphere.scale.y = 0.21;
        sphere.scale.z = 0.21;
        sphere.color.r = pred.is_in_court ? 0.0f : 1.0f;
        sphere.color.g = pred.is_in_court ? 1.0f : 0.0f;
        sphere.color.b = 0.0f;
        sphere.color.a = 0.9f;
        markers.markers.push_back(sphere);

        marker_pub_->publish(markers);
    }

    volleyball_tracker::AeroEKF ekf_;
    volleyball_tracker::TrajectoryPredictor predictor_;
    std::string frame_id_;
    double target_z_{0.8};
    int consecutive_lost_{0};
    rclcpp::Time last_meas_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr raw_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr filtered_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr landing_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VolleyballTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
