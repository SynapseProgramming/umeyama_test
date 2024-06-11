#include <umeyama_test/umeyama.hpp>

Umeyama::Umeyama() : Node("umeyama")
{
    m_timer = this->create_wall_timer(
        500ms, std::bind(&Umeyama::timerCallback, this));

    m_tf_broadcaster =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    m_source_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "source_cloud", 10);

    m_target_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "target_cloud", 10);
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

    Eigen::Translation3f translation(1.0, 0.0, 1.0);
    Eigen::Quaternionf rotation(q.w(), q.x(), q.y(), q.z());
    m_transform = (translation * rotation).matrix();

    std::vector<double> y_coord = {0.5, 0.5, -0.5, -0.5};
    std::vector<double> z_coord = {0.5, 0, 0, 0.5};
    double x_coord = 1.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr rect_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < 4; i++)
    {
        pcl::PointXYZ point;
        point.x = x_coord;
        point.y = y_coord[i];
        point.z = z_coord[i];
        rect_cloud->push_back(point);
    }
    // settle target cloud
    toRosPcl(rect_cloud, m_target_cloud);

    // source cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    transformCloud(rect_cloud, trans_cloud, m_transform);

    Eigen::Matrix4f final_transformation;
    const pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,
                                                         pcl::PointXYZ>
        svd_est(false);

    svd_est.estimateRigidTransformation(*trans_cloud, *rect_cloud, final_transformation);

    // print out the final transformation
    double x_translation = final_transformation(0, 3);
    double y_translation = final_transformation(1, 3);
    double z_translation = final_transformation(2, 3);

    Eigen::Matrix3f correction_rotation_matrix =
        final_transformation.block<3, 3>(0, 0);

    Eigen::Matrix3f original_rotation_matrix =
        m_transform.block<3, 3>(0, 0);

    Eigen::Matrix3f corrected_rotation_matrix =
        correction_rotation_matrix * original_rotation_matrix;

    std::cout << "x_translation: " << x_translation << std::endl;
    std::cout << "y_translation: " << y_translation << std::endl;
    std::cout << "z_translation: " << z_translation << std::endl;

    // corrected translation
    Eigen::Vector3f corrected_translation = correction_rotation_matrix * translation.vector() + Eigen::Vector3f(x_translation, y_translation, z_translation);
    // print out the correction translation
    std::cout << "corrected x_translation: " << corrected_translation(0) << std::endl;
    std::cout << "corrected y_translation: " << corrected_translation(1) << std::endl;
    std::cout << "corrected z_translation: " << corrected_translation(2) << std::endl;

    // print out rpy values of corrected rotation matrix
    Eigen::Vector3f euler_angles = corrected_rotation_matrix.eulerAngles(2, 1, 0);

    float yaw = euler_angles[0];
    float pitch = euler_angles[1];
    float roll = euler_angles[2];
    std::cout << "roll: " << roll << " pitch: " << pitch << " yaw: " << yaw
              << std::endl;

    toRosPcl(rect_cloud, m_source_cloud);
}

void Umeyama::timerCallback()
{

    m_t.header.stamp = this->get_clock()->now();

    m_tf_broadcaster->sendTransform(m_t);
    m_target_cloud.header.stamp = this->get_clock()->now();
    m_source_cloud.header.stamp = this->get_clock()->now();

    m_target_cloud.header.frame_id = "target_frame";
    m_source_cloud.header.frame_id = "source_frame";

    m_source_pub->publish(m_source_cloud);
    m_target_pub->publish(m_target_cloud);
}

void Umeyama::transformCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &transformed_cloud,
    const Eigen::Matrix4f &transform)
{
    for (auto &point : cloud->points)
    {
        Eigen::Vector4f point_vector(point.x, point.y, point.z, 1.0);
        Eigen::Vector4f transformed_point = transform * point_vector;

        double px = transformed_point(0);
        double py = transformed_point(1);
        double pz = transformed_point(2);

        pcl::PointXYZ pcl_point;
        pcl_point.x = px;
        pcl_point.y = py;
        pcl_point.z = pz;
        transformed_cloud->push_back(pcl_point);
    }
}

void Umeyama::toRosPcl(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_cloud, sensor_msgs::msg::PointCloud2 &ros_cloud)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*pcl_cloud, pcl_pc2);
    pcl_conversions::fromPCL(pcl_pc2, ros_cloud);
}