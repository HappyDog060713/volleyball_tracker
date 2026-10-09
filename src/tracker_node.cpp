#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "volleyball_tracker/aero_ekf.hpp"
#include "volleyball_tracker/trajectory_predictor.hpp"
#include "volleyball_tracker/msg/trajectory_prediction.hpp"

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
        this->declare_parameter<std::string>("tracking_frame", "base_link");

        // Fallback extrinsic parameters if TF is unavailable
        this->declare_parameter<double>("cam_mount_x", 0.3);
        this->declare_parameter<double>("cam_mount_y", 0.0);
        this->declare_parameter<double>("cam_mount_z", 0.8);
        this->declare_parameter<double>("cam_pitch_deg", 10.0);

        tracking_frame_ = this->get_parameter("tracking_frame").as_string();
        target_z_ = this->get_parameter("target_interception_z").as_double();

        cam_mount_x_ = this->get_parameter("cam_mount_x").as_double();
        cam_mount_y_ = this->get_parameter("cam_mount_y").as_double();
        cam_mount_z_ = this->get_parameter("cam_mount_z").as_double();
        cam_pitch_rad_ = this->get_parameter("cam_pitch_deg").as_double() * M_PI / 180.0;

        ekf_.setNoiseParams(
            this->get_parameter("q_pos").as_double(),
            this->get_parameter("q_vel").as_double(),
            this->get_parameter("r_pos").as_double());

        predictor_.setCourtDimensions(
            this->get_parameter("court_x_min").as_double(),
            this->get_parameter("court_x_max").as_double(),
            this->get_parameter("court_y_min").as_double(),
            this->get_parameter("court_y_max").as_double());

        // TF2 setup
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        raw_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/volleyball/position", rclcpp::SensorDataQoS(),
            std::bind(&VolleyballTrackerNode::positionCallback, this, std::placeholders::_1));

        filtered_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/volleyball/position_filtered", rclcpp::SensorDataQoS());

        trajectory_pred_pub_ = this->create_publisher<volleyball_tracker::msg::TrajectoryPrediction>(
            "/volleyball/trajectory_pred", 10);

        landing_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/volleyball/landing_prediction", 10);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/volleyball/trajectory_markers", 10);

        last_time_ = this->now();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), // 100 Hz
            std::bind(&VolleyballTrackerNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "VolleyballTrackerNode initialized. Target tracking frame: %s (Z upward)",
                    tracking_frame_.c_str());
    }

private:
    // Transforms point from camera optical frame (Z forward, X right, Y down) to base_link (X forward, Y left, Z upward)
    Eigen::Vector3d transformOpticalToBase(const geometry_msgs::msg::PointStamped& raw_pt) {
        if (raw_pt.header.frame_id != tracking_frame_) {
            try {
                if (tf_buffer_->canTransform(tracking_frame_, raw_pt.header.frame_id, tf2::TimePointZero)) {
                    geometry_msgs::msg::PointStamped transformed;
                    tf_buffer_->transform(raw_pt, transformed, tracking_frame_);
                    return Eigen::Vector3d(transformed.point.x, transformed.point.y, transformed.point.z);
                }
            } catch (const tf2::TransformException&) {
                // Fall through to manual extrinsic matrix
            }

            // Fallback manual transform from camera optical frame to base_link
            double x_opt = raw_pt.point.x;
            double y_opt = raw_pt.point.y;
            double z_opt = raw_pt.point.z;

            // Optical to standard robot camera link: X_fwd = Z_opt, Y_left = -X_opt, Z_up = -Y_opt
            // Then rotate by pitch:
            double x_base = z_opt * std::cos(cam_pitch_rad_) - y_opt * std::sin(cam_pitch_rad_) + cam_mount_x_;
            double y_base = -x_opt + cam_mount_y_;
            double z_base = -z_opt * std::sin(cam_pitch_rad_) - y_opt * std::cos(cam_pitch_rad_) + cam_mount_z_;

            return Eigen::Vector3d(x_base, y_base, z_base);
        }

        return Eigen::Vector3d(raw_pt.point.x, raw_pt.point.y, raw_pt.point.z);
    }

    void positionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        rclcpp::Time current_time = msg->header.stamp;
        Eigen::Vector3d z_in_base = transformOpticalToBase(*msg);

        if (!ekf_.isInitialized()) {
            ekf_.initialize(z_in_base);
            last_meas_time_ = current_time;
            is_tracking_ = true;
            return;
        }

        double dt = (current_time - last_meas_time_).seconds();
        if (dt > 0.0 && dt < 0.2) {
            ekf_.predict(dt);
        }

        double gate = this->get_parameter("gate_thresh").as_double();
        if (ekf_.update(z_in_base, gate)) {
            last_meas_time_ = current_time;
            is_tracking_ = true;
            publishFilteredState(current_time);
        }
    }

    void timerCallback() {
        rclcpp::Time now = this->now();
        if (!ekf_.isInitialized()) {
            publishLostState(now);
            return;
        }

        double time_since_meas = (now - last_meas_time_).seconds();
        if (time_since_meas > 0.5) {
            // Lost ball tracking after 0.5s
            ekf_.reset();
            is_tracking_ = false;
            publishLostState(now);
            return;
        }

        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt > 0.0 && dt < 0.05) {
            ekf_.predict(dt);
        }

        publishFilteredState(now);
    }

    void publishLostState(const rclcpp::Time& stamp) {
        volleyball_tracker::msg::TrajectoryPrediction pred_msg;
        pred_msg.header.stamp = stamp;
        pred_msg.header.frame_id = tracking_frame_;
        pred_msg.is_tracking = false;
        pred_msg.is_in_court = false;
        pred_msg.time_to_landing = -1.0f;
        trajectory_pred_pub_->publish(pred_msg);
    }

    void publishFilteredState(const rclcpp::Time& stamp) {
        Eigen::Vector3d pos = ekf_.getPosition();
        Eigen::Vector3d vel = ekf_.getVelocity();

        // 1. Publish filtered position
        geometry_msgs::msg::PointStamped filt_msg;
        filt_msg.header.stamp = stamp;
        filt_msg.header.frame_id = tracking_frame_;
        filt_msg.point.x = pos.x();
        filt_msg.point.y = pos.y();
        filt_msg.point.z = pos.z();
        filtered_pub_->publish(filt_msg);

        // 2. Predict landing and trajectory
        auto pred = predictor_.predictLanding(pos, vel, target_z_);

        volleyball_tracker::msg::TrajectoryPrediction pred_msg;
        pred_msg.header.stamp = stamp;
        pred_msg.header.frame_id = tracking_frame_;
        pred_msg.current_position = filt_msg.point;
        pred_msg.current_velocity.x = vel.x();
        pred_msg.current_velocity.y = vel.y();
        pred_msg.current_velocity.z = vel.z();
        pred_msg.is_tracking = true;

        if (pred.is_valid) {
            pred_msg.landing_position.x = pred.landing_pos.x();
            pred_msg.landing_position.y = pred.landing_pos.y();
            pred_msg.landing_position.z = pred.landing_pos.z();
            pred_msg.landing_velocity.x = pred.landing_vel.x();
            pred_msg.landing_velocity.y = pred.landing_vel.y();
            pred_msg.landing_velocity.z = pred.landing_vel.z();
            pred_msg.time_to_landing = static_cast<float>(pred.time_to_landing);
            pred_msg.is_in_court = pred.is_in_court;

            // Also publish PointStamped for simple tools
            geometry_msgs::msg::PointStamped land_pt;
            land_pt.header.stamp = stamp;
            land_pt.header.frame_id = tracking_frame_;
            land_pt.point = pred_msg.landing_position;
            landing_pub_->publish(land_pt);

            publishMarkers(pred, stamp);
        } else {
            pred_msg.time_to_landing = -1.0f;
            pred_msg.is_in_court = false;
        }

        trajectory_pred_pub_->publish(pred_msg);
    }

    void publishMarkers(const volleyball_tracker::TrajectoryPredictionResult& pred, const rclcpp::Time& stamp) {
        visualization_msgs::msg::MarkerArray markers;

        // Line strip for trajectory
        visualization_msgs::msg::Marker line_strip;
        line_strip.header.frame_id = tracking_frame_;
        line_strip.header.stamp = stamp;
        line_strip.ns = "trajectory";
        line_strip.id = 0;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.scale.x = 0.03;
        line_strip.color.r = 1.0;
        line_strip.color.g = 0.55;
        line_strip.color.b = 0.0;
        line_strip.color.a = 0.85;

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
        sphere.header.frame_id = tracking_frame_;
        sphere.header.stamp = stamp;
        sphere.ns = "landing";
        sphere.id = 1;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position.x = pred.landing_pos.x();
        sphere.pose.position.y = pred.landing_pos.y();
        sphere.pose.position.z = pred.landing_pos.z();
        sphere.scale.x = 0.22;
        sphere.scale.y = 0.22;
        sphere.scale.z = 0.22;
        sphere.color.r = pred.is_in_court ? 0.0f : 1.0f;
        sphere.color.g = pred.is_in_court ? 1.0f : 0.0f;
        sphere.color.b = 0.1f;
        sphere.color.a = 0.9f;
        markers.markers.push_back(sphere);

        marker_pub_->publish(markers);
    }

    volleyball_tracker::AeroEKF ekf_;
    volleyball_tracker::TrajectoryPredictor predictor_;
    std::string tracking_frame_;
    double target_z_{0.8};

    double cam_mount_x_{0.3};
    double cam_mount_y_{0.0};
    double cam_mount_z_{0.8};
    double cam_pitch_rad_{0.0};

    bool is_tracking_{false};
    rclcpp::Time last_meas_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr raw_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr filtered_pub_;
    rclcpp::Publisher<volleyball_tracker::msg::TrajectoryPrediction>::SharedPtr trajectory_pred_pub_;
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
