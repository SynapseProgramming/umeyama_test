#include <umeyama_test/umeyama.hpp>

Umeyama::Umeyama() : Node("umeyama")
{
    m_timer = this->create_wall_timer(
        500ms, std::bind(&Umeyama::timerCallback, this));

    m_tf_broadcaster =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    m_source_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "source_cloud", 10);
    m_t.transform.translation.x = 1.0;
    m_t.transform.translation.y = 0;
    m_t.transform.translation.z = 1;

    tf2::Quaternion q;
    q.setRPY(0.1, 0.2, 0.3);
    m_t.transform.rotation.x = q.x();
    m_t.transform.rotation.y = q.y();
    m_t.transform.rotation.z = q.z();
    m_t.transform.rotation.w = q.w();
    m_t.header.frame_id = "target_frame";
    m_t.child_frame_id = "source_frame";

    std::vector<double> y_coord = {0.5, 0.5, -0.5, -0.5};
    std::vector<double> z_coord = {0.5, 0, 0, 0.5};
    double x_coord = 1.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < 4; i++)
    {
        pcl::PointXYZ point;
        point.x = x_coord;
        point.y = y_coord[i];
        point.z = z_coord[i];
        cloud->push_back(point);
    }
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*cloud, pcl_pc2);
    m_rect_cloud = sensor_msgs::msg::PointCloud2();
    pcl_conversions::fromPCL(pcl_pc2, m_rect_cloud);
}

void Umeyama::timerCallback()
{

    m_t.header.stamp = this->get_clock()->now();

    m_tf_broadcaster->sendTransform(m_t);
    m_rect_cloud.header.stamp = this->get_clock()->now();
    m_rect_cloud.header.frame_id = "source_frame";
    m_source_pub->publish(m_rect_cloud);
}