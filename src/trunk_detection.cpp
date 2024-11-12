#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include "yolo_tensor_cpp/yolov8.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "hand_interfaces/srv/frond_position.hpp"  // Adjust this include according to your package structure
#include <vector>

class YoloV8Node : public rclcpp::Node {
public:
    YoloV8Node(const std::string &onnxModelPath, const YoloV8Config &config)
        : Node("yolo_v8_node"), yoloV8(onnxModelPath, config) {
        // Subscribe to the ZED image topic for RGB images  /zed/zed_node/left/image_rect_color
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 
            10, 
            std::bind(&YoloV8Node::imageCallback, this, std::placeholders::_1)
        );

        // Subscribe to the ZED depth topic for depth images "/zed/zed_node/depth/depth_registered"
        depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 
            10, 
            std::bind(&YoloV8Node::depthCallback, this, std::placeholders::_1)
        );

        // Create service server
        service_ = this->create_service<hand_interfaces::srv::FrondPosition>(
            "/frond_pos",
            std::bind(&YoloV8Node::handleGetFrondPosition, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS image to OpenCV format
        img_ = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Run inference
        const auto objects = yoloV8.detectObjects(img_);

        // Draw the bounding boxes on the image
        yoloV8.drawObjectLabels(img_, objects);

        // Store the midpoints of bounding boxes
        mid_x_.clear();
        mid_y_.clear();

        for (const auto &obj : objects) {
            // Bounding box coordinates
            int x0 = obj.rect.x;
            int y0 = obj.rect.y;
            int x1 = x0 + obj.rect.width;
            int y1 = y0 + obj.rect.height;

            // Calculate the midpoint
            int mid_x = (x0 + x1) / 2;
            int mid_y = (y0 + y1) / 2;

            // mid_x_.push_back(mid_x);
            mid_x_.push_back(mid_x);
            mid_y_.push_back(y1);

            std::cout << "Midpoint (x0, x1, y0, y1): (" << x0 << ", " << x1 << ", " << y0 <<", " << y1 << ")" << std::endl;
        }

        // Display the image in a window
        cv::imshow("YOLOv8 Detection", img_);
        cv::waitKey(1); // 1 ms delay to allow OpenCV to process events
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr depthMsg) {
        if (mid_x_.empty() || mid_y_.empty()) {
            RCLCPP_WARN(get_logger(), "No bounding box midpoints available for depth processing.");
            return;
        }

        // Convert ROS depth image to raw depth data
        const float *depths = reinterpret_cast<const float *>(&depthMsg->data[0]);

        // Ensure depth image dimensions match
        if (depthMsg->width == 0 || depthMsg->height == 0) {
            RCLCPP_ERROR(get_logger(), "Depth image has invalid dimensions.");
            return;
        }

        // Initialize nearest object variables
        int nearest_index = -1;
        float min_depth = std::numeric_limits<float>::max(); // Start with a large value

        // Process each bounding box midpoint
        for (size_t i = 0; i < mid_x_.size(); ++i) {
            int mid_x = mid_x_[i];
            int mid_y = mid_y_[i];

            // Ensure mid_x and mid_y are within depth image bounds
            if (mid_x >= 0 && mid_x < static_cast<int>(depthMsg->width) && mid_y >= 0 && mid_y < static_cast<int>(depthMsg->height)) {
                int centerIdx = mid_x + depthMsg->width * mid_y;
                float depth = depths[centerIdx];

                // Check if this object is the nearest one
                if (depth < min_depth) {
                    min_depth = depth;
                    nearest_index = i; // Update the nearest index
                }
            } else {
                RCLCPP_WARN(get_logger(), "Midpoint out of depth image bounds for object %zu: (x, y) = (%d, %d)", i, mid_x, mid_y);
            }
        }

        // If we found a nearest object, calculate its 3D coordinates
        if (nearest_index != -1) {
            int mid_x = mid_x_[nearest_index];
            int mid_y = mid_y_[nearest_index];
            float depth = min_depth;

            // Apply the pinhole camera model to compute real-world X, Y coordinates
            float X = (mid_x - cx_) * depth / fx_;
            float Y = -1*(mid_y - cy_) * depth / fy_;
            float Z = depth;  // Z-coordinate is the depth value

            // if (Z >=0.8 || Z <=0.2){
            // Store the nearest object's pose
            nearest_object_pose_.header.stamp = this->now();
            nearest_object_pose_.header.frame_id = "zed_camera_center"; // Adjust frame id if needed
            nearest_object_pose_.pose.position.x = Z; // depth
            nearest_object_pose_.pose.position.y = -X; // horizontal
            nearest_object_pose_.pose.position.z = Y; // vertical
            // }
            // else{
            // RCLCPP_INFO(get_logger(), "The nearest object is not within 0.2m <--> 0.8m");
            // }

            RCLCPP_INFO(get_logger(), "Nearest Object: (X, Y, Z): (%f, %f, %f)", X, Y, Z);
        }
    }

    void handleGetFrondPosition(const std::shared_ptr<hand_interfaces::srv::FrondPosition::Request> request,
                                 std::shared_ptr<hand_interfaces::srv::FrondPosition::Response> response) {
        
        response->pose = nearest_object_pose_;
        response->success= true;
        RCLCPP_INFO(get_logger(), "Providing nearest frond position");

    }

    YoloV8 yoloV8; // YOLOv8 instance
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
    rclcpp::Service<hand_interfaces::srv::FrondPosition>::SharedPtr service_;

    cv::Mat img_;  // Holds the RGB image
    std::vector<int> mid_x_;  // Stores midpoints for detected objects
    std::vector<int> mid_y_;  // Stores midpoints for detected objects

    geometry_msgs::msg::PoseStamped nearest_object_pose_; // Store nearest object's pose

    // ZED Camera intrinsic parameters for 1280x720 resolution
    const float fx_ = 529.836;
    const float fy_ = 529.836;
    const float cx_ = 629.956;
    const float cy_ = 367.842;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    
    YoloV8Config config;
    std::string onnxModelPath;

    // Ensure the model path is provided as an argument
    if (argc != 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: ros2 run yolo_tensor_cpp detect_object_video <model_path>");
        return -1;
    }
    
    onnxModelPath = argv[1]; // Get the model path from command line argument

    // Create the YoloV8 Node
    auto node = std::make_shared<YoloV8Node>(onnxModelPath, config);
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
