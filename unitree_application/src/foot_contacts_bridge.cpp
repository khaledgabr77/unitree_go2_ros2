#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <champ_msgs/msg/contacts_stamped.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>

// Collects the per foot contact sensors bridged out of Gazebo and republishes
// them as the single champ_msgs/ContactsStamped that state_estimation_node
// expects.
class FootContactsBridge : public rclcpp::Node
{
    std::vector<std::string> contact_topics_;
    double publish_rate_;
    double contact_timeout_;
    double min_normal_force_;

    std::vector<bool> in_contact_;
    std::vector<rclcpp::Time> last_msg_time_;

    std::vector<rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr> contact_subscriptions_;
    rclcpp::Publisher<champ_msgs::msg::ContactsStamped>::SharedPtr foot_contacts_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

  public:
    FootContactsBridge() : Node("foot_contacts_bridge")
    {
        // the order of the topics defines the order of the published booleans,
        // and champ::QuadrupedBase registers its legs as lf, rf, lh, rh
        contact_topics_ = this->declare_parameter<std::vector<std::string>>(
            "contact_topics", {"lf_foot_contacts", "rf_foot_contacts", "lh_foot_contacts", "rh_foot_contacts"});
        publish_rate_ = this->declare_parameter("publish_rate", 100.0);
        contact_timeout_ = this->declare_parameter("contact_timeout", 0.05);
        min_normal_force_ = this->declare_parameter("min_normal_force", 0.0);

        if (publish_rate_ <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "publish_rate must be positive, falling back to 100 Hz");
            publish_rate_ = 100.0;
        }

        const rclcpp::Time never(0, 0, this->get_clock()->get_clock_type());
        in_contact_.assign(contact_topics_.size(), false);
        last_msg_time_.assign(contact_topics_.size(), never);

        for (size_t i = 0; i < contact_topics_.size(); i++)
        {
            contact_subscriptions_.push_back(this->create_subscription<ros_gz_interfaces::msg::Contacts>(
                contact_topics_[i], 10,
                [this, i](const ros_gz_interfaces::msg::Contacts::SharedPtr msg) { this->contactCallback_(i, msg); }));
        }

        foot_contacts_publisher_ = this->create_publisher<champ_msgs::msg::ContactsStamped>("foot_contacts", 10);

        publish_timer_ = this->create_timer(std::chrono::duration<double>(1.0 / publish_rate_),
                                            std::bind(&FootContactsBridge::publishContacts_, this));
    }

  private:
    bool hasContact_(const ros_gz_interfaces::msg::Contacts &msg) const
    {
        // Gazebo publishes contacts only when they happen, just getting a msg tells that there are contacts,
        // so no need for additional checking without threshold (min_normal_force_ <= 0.0).
        if (min_normal_force_ <= 0.0)
            return true;

        // The penetration depths Gazebo reports for the feet are numerically zero, so the only usable
        // measure of how firmly a foot is planted is the contact wrench
        for (const auto &contact : msg.contacts)
        {
            for (const auto &wrench : contact.wrenches)
            {
                const auto &force = wrench.body_1_wrench.force;
                const double magnitude = std::sqrt(force.x * force.x + force.y * force.y + force.z * force.z);

                if (magnitude >= min_normal_force_)
                    return true;
            }
        }
        return false;
    }

    void contactCallback_(size_t leg, const ros_gz_interfaces::msg::Contacts::SharedPtr msg)
    {
        in_contact_[leg] = hasContact_(*msg);
        last_msg_time_[leg] = msg->header.stamp;
    }

    void publishContacts_()
    {
        const rclcpp::Time now = this->get_clock()->now();
        rclcpp::Time newest(0, 0, now.get_clock_type());

        champ_msgs::msg::ContactsStamped contacts_msg;
        contacts_msg.contacts.resize(contact_topics_.size());

        for (size_t i = 0; i < contact_topics_.size(); i++)
        {
            // a sensor that stops reporting means the foot left the ground
            const bool fresh =
                last_msg_time_[i].nanoseconds() > 0 && (now - last_msg_time_[i]).seconds() < contact_timeout_;

            contacts_msg.contacts[i] = fresh && in_contact_[i];

            if (last_msg_time_[i] > newest)
                newest = last_msg_time_[i];
        }

        // timestamp has to keep advancing for state_estimation node to work even when every foot is airborne,
        // so update timestamp with `now` if `last_msg_time_` is too old
        const bool newest_is_fresh = newest.nanoseconds() > 0 && (now - newest).seconds() < contact_timeout_;
        contacts_msg.header.stamp = newest_is_fresh ? newest : now;

        foot_contacts_publisher_->publish(contacts_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FootContactsBridge>());
    rclcpp::shutdown();

    return 0;
}
