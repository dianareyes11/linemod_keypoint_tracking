#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <glm/glm.hpp>

#include "defines.h"

struct KeypointTemplate
{
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	std::vector<cv::Point3f> objectPoints;
	cv::Rect bbox;
	cv::Size imageSize;
};

class KeypointDetector
{
public:
	KeypointDetector();

	void addTemplate(const cv::Mat& color, const cv::Mat& mask, const cv::Mat& depth,
	                 const CameraParameters& cam_params, const glm::vec3& cam_pos);
	void pushBackTemplates(const std::string& class_id);
	void write(const std::string& filename) const;
	bool read(const std::string& filename);

	bool detect(const cv::Mat& color, const std::string& class_id,
	            std::vector<cv::Rect>& out_boxes) const;
	bool estimatePose(const cv::Mat& color, const std::string& class_id,
	                  const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,
	                  cv::Rect& out_box, cv::Mat& out_rvec, cv::Mat& out_tvec,
	                  int& out_inliers) const;

private:
	struct Candidate
	{
		cv::Rect box;
		int inliers = 0;
	};

	cv::Ptr<cv::ORB> orb_;
	std::vector<KeypointTemplate> current_templates_;
	std::vector<std::vector<KeypointTemplate>> templates_;
	std::vector<std::string> class_ids_;

	int min_inliers_;
	float ratio_thresh_;
	float nms_iou_thresh_;
	uint32_t max_boxes_;

	static bool computeBBoxFromMask(const cv::Mat& mask, cv::Rect& bbox);
	static float iou(const cv::Rect& a, const cv::Rect& b);
	static void computeCameraPose(const glm::vec3& cam_pos, glm::mat3& out_r,
	                              glm::vec3& out_t);
};
