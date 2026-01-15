#include <memory>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/opencv.hpp>

#include "PoseDetection.h"
#include "utility.h"

using sensor_msgs::msg::CompressedImage;

class LinemodDetectorNode : public rclcpp::Node
{
public:
  LinemodDetectorNode()
  : Node("linemod_detector_node"),
    detector_(),
    color_sub_(this, declare_parameter("color_topic", std::string("/camera/color/image_raw/compressed"))),
    depth_sub_(this, declare_parameter("depth_topic", std::string("/camera/depth/image_raw/compressed"))),
    sync_(SyncPolicy(10), color_sub_, depth_sub_)
  {
    class_name_ = declare_parameter("class_name", std::string("milk_carton.ply"));
    display_ = declare_parameter("display", true);
    readSettings(cam_params_, template_settings_);
    target_size_ = cv::Size(cam_params_.videoWidth, cam_params_.videoHeight);

    sync_.registerCallback(
      std::bind(&LinemodDetectorNode::onSync, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "LINEMOD detector node started.");
    RCLCPP_INFO(get_logger(), "Using class_name=%s", class_name_.c_str());
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<CompressedImage, CompressedImage>;

  void onSync(const CompressedImage::ConstSharedPtr &color_msg,
              const CompressedImage::ConstSharedPtr &depth_msg)
  {
    cv::Mat color = decodeCompressed(*color_msg, cv::IMREAD_COLOR, "color");
    if (color.empty())
    {
      RCLCPP_WARN(get_logger(), "Failed to decode color image");
      return;
    }

    cv::Mat depth = decodeCompressed(*depth_msg, cv::IMREAD_UNCHANGED, "depth");
    if (depth.empty())
    {
      RCLCPP_WARN(get_logger(), "Failed to decode depth image");
      return;
    }

    if (depth.type() == CV_32FC1)
    {
      cv::Mat depth_mm;
      depth.convertTo(depth_mm, CV_16UC1, 1000.0);
      depth = depth_mm;
    }
    else if (depth.type() != CV_16UC1)
    {
      RCLCPP_WARN(get_logger(), "Unexpected depth type: %d", depth.type());
      return;
    }

    if (color.size() != target_size_)
    {
      cv::resize(color, color, target_size_, 0.0, 0.0, cv::INTER_AREA);
    }
    if (depth.size() != target_size_)
    {
      cv::resize(depth, depth, target_size_, 0.0, 0.0, cv::INTER_NEAREST);
    }

    std::vector<cv::Mat> imgs;
    imgs.push_back(color);
    imgs.push_back(depth);

    std::vector<ObjectPose> poses;
    detector_.detect(imgs, class_name_, 1, poses, display_);
  }

  cv::Mat decodeCompressed(const CompressedImage &msg, int flags, const char *label)
  {
    if (msg.data.empty())
    {
      RCLCPP_WARN(get_logger(), "%s image data is empty", label);
      return cv::Mat();
    }
    cv::Mat raw(1, static_cast<int>(msg.data.size()), CV_8UC1, const_cast<uint8_t *>(msg.data.data()));
    return cv::imdecode(raw, flags);
  }

  PoseDetection detector_;
  std::string class_name_;
  bool display_ = true;
  CameraParameters cam_params_;
  TemplateGenerationSettings template_settings_;
  cv::Size target_size_;

  message_filters::Subscriber<CompressedImage> color_sub_;
  message_filters::Subscriber<CompressedImage> depth_sub_;
  message_filters::Synchronizer<SyncPolicy> sync_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LinemodDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
