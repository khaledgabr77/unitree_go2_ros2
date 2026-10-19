#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Republishes a TF chain as nav_msgs/Odometry
// so it can be recorded and compared against /odom/ground_truth.
class TfToOdom : public rclcpp::Node
{
    std::string map_frame_;
    std::string child_frame_;
    double publish_rate_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

  public:
    TfToOdom() : Node("tf_to_odom")
    {
        map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
        child_frame_ = this->declare_parameter<std::string>("child_frame", "base_link");
        publish_rate_ = this->declare_parameter("publish_rate", 20.0);

        if (publish_rate_ <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "publish_rate must be positive, falling back to 20 Hz");
            publish_rate_ = 20.0;
        }

        // the node's clock honours use_sim_time, so lookups and stamps stay on sim time
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("slam_pose", 10);

        publish_timer_ = this->create_timer(std::chrono::duration<double>(1.0 / publish_rate_),
                                            std::bind(&TfToOdom::publishOdometry_, this));
    }

  private:
    void publishOdometry_()
    {
        geometry_msgs::msg::TransformStamped transform;

        try
        {
            transform = tf_buffer_->lookupTransform(map_frame_, child_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            // expected until the SLAM back-end activates and starts publishing map->odom
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "%s -> %s not available yet: %s",
                                 map_frame_.c_str(), child_frame_.c_str(), ex.what());
            return;
        }

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = transform.header.stamp;
        odom_msg.header.frame_id = map_frame_;
        odom_msg.child_frame_id = child_frame_;

        odom_msg.pose.pose.position.x = transform.transform.translation.x;
        odom_msg.pose.pose.position.y = transform.transform.translation.y;
        odom_msg.pose.pose.position.z = transform.transform.translation.z;
        odom_msg.pose.pose.orientation = transform.transform.rotation;

        // twist and covariance are left zero, evo only reads the pose
        odom_publisher_->publish(odom_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfToOdom>());
    rclcpp::shutdown();

    return 0;
}
