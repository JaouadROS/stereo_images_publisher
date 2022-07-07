#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <publisher/util.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

int main(int argc, char* argv[]) {

    std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);
    std::string img_dir_path = argv[1];
    std::string img_left_folder_name = argv[2];
    std::string img_right_folder_name = argv[3];
    unsigned int img_fps = std::atoi(argv[4]);

    // initialize this node
    auto node = std::make_shared<rclcpp::Node>("image_publisher");
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    custom_qos.depth = 1;

    auto publisher_left = image_transport::create_publisher(node.get(), "camera/left/image_raw", custom_qos);
    auto publisher_right = image_transport::create_publisher(node.get(), "camera/right/image_raw", custom_qos);

    sensor_msgs::msg::Image::SharedPtr msg_left, msg_right;
    rclcpp::WallRate pub_rate(img_fps);
    rclcpp::executors::SingleThreadedExecutor exec;

    exec.add_node(node);

    images_sequence sequence(img_dir_path, img_left_folder_name, img_right_folder_name);
    const auto frames = sequence.get_frames();

    for (unsigned int i = 0; i < frames.size(); ++i) {

        const auto& frame = frames.at(i);
        const cv::Mat img_left = cv::imread(frame.left_img_path_, cv::IMREAD_GRAYSCALE);
        const cv::Mat img_right = cv::imread(frame.right_img_path_, cv::IMREAD_GRAYSCALE);

        msg_left = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", img_left).toImageMsg();
        msg_left->header.stamp = node->now();
        msg_left->header.frame_id = "camera_link";

        msg_right = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", img_right).toImageMsg();
        msg_right->header.stamp = msg_left->header.stamp;
        msg_right->header.frame_id = "camera_link";

        publisher_left.publish(msg_left);
        publisher_right.publish(msg_right);

        exec.spin_some();
        pub_rate.sleep();
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
