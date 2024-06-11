#include <umeyama_test/umeyama.hpp>

Umeyama::Umeyama() : Node("umeyama")
{
    m_timer = this->create_wall_timer(
        500ms, std::bind(&Umeyama::timerCallback, this));

    m_tf_broadcaster =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void Umeyama::timerCallback()
{
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "target_frame";
    t.child_frame_id = "source_frame";

    t.transform.translation.x = 1.0;
    t.transform.translation.y = 0;
    t.transform.translation.z = 1;

    tf2::Quaternion q;
    q.setRPY(0.1, 0.2, 0.3);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    m_tf_broadcaster->sendTransform(t);
}