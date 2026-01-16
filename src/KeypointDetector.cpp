#include "KeypointDetector.h"

#include <algorithm>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/rotate_vector.hpp>

KeypointDetector::KeypointDetector() :
	orb_(cv::ORB::create(600)),
	min_inliers_(12),
	ratio_thresh_(0.75f),
	nms_iou_thresh_(0.3f),
	max_boxes_(3)
{
}

bool KeypointDetector::computeBBoxFromMask(const cv::Mat& mask, cv::Rect& bbox)
{
	std::vector<cv::Point> pts;
	cv::findNonZero(mask, pts);
	if (pts.empty())
	{
		return false;
	}
	bbox = cv::boundingRect(pts);
	return true;
}

float KeypointDetector::iou(const cv::Rect& a, const cv::Rect& b)
{
	const int x1 = std::max(a.x, b.x);
	const int y1 = std::max(a.y, b.y);
	const int x2 = std::min(a.x + a.width, b.x + b.width);
	const int y2 = std::min(a.y + a.height, b.y + b.height);
	const int inter_w = std::max(0, x2 - x1);
	const int inter_h = std::max(0, y2 - y1);
	const int inter = inter_w * inter_h;
	const int uni = a.area() + b.area() - inter;
	if (uni <= 0)
	{
		return 0.0f;
	}
	return static_cast<float>(inter) / static_cast<float>(uni);
}

void KeypointDetector::computeCameraPose(const glm::vec3& cam_pos, glm::mat3& out_r,
                                         glm::vec3& out_t)
{
	glm::vec3 up(0.0f, 1.0f, 0.0f);
	glm::vec3 cam = cam_pos;
	if (cam.x == 0.0f && cam.z == 0.0f)
	{
		cam.x = 0.00000000001f;
	}
	glm::vec3 camUp = glm::normalize(glm::cross(cam, glm::cross(cam, up)));
	glm::vec3 rotatedUp = glm::rotate(-camUp, glm::radians(0.0f), glm::normalize(cam));
	glm::mat4 view = glm::lookAt(cam, glm::vec3(0.0f), rotatedUp);

	glm::mat4 coordinateTransform(1.0f);
	coordinateTransform[1][1] = -1.0f;
	coordinateTransform[2][2] = -1.0f;
	glm::mat4 viewCv = glm::transpose(glm::transpose(view) * coordinateTransform);
	glm::qua<float> quat = glm::toQuat(viewCv);
	out_r = glm::mat3_cast(quat);
	out_t = glm::vec3(0.0f, 0.0f, glm::length(cam_pos));
}

void KeypointDetector::addTemplate(const cv::Mat& color, const cv::Mat& mask,
                                   const cv::Mat& depth, const CameraParameters& cam_params,
                                   const glm::vec3& cam_pos)
{
	cv::Mat gray;
	if (color.channels() == 3)
	{
		cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
	}
	else
	{
		gray = color;
	}

	cv::Rect bbox;
	if (!computeBBoxFromMask(mask, bbox))
	{
		return;
	}

	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	orb_->detectAndCompute(gray, mask, keypoints, descriptors);
	if (descriptors.empty())
	{
		return;
	}

	glm::mat3 rot;
	glm::vec3 trans;
	computeCameraPose(cam_pos, rot, trans);

	std::vector<cv::KeyPoint> filtered_kp;
	cv::Mat filtered_desc;
	std::vector<cv::Point3f> obj_points;
	for (size_t i = 0; i < keypoints.size(); ++i)
	{
		const cv::Point2f& pt = keypoints[i].pt;
		const int u = std::lround(pt.x);
		const int v = std::lround(pt.y);
		if (u < 0 || v < 0 || u >= depth.cols || v >= depth.rows)
		{
			continue;
		}
		const uint16_t z = depth.at<uint16_t>(v, u);
		if (z == 0)
		{
			continue;
		}

		const float zf = static_cast<float>(z);
		const float xc = (pt.x - cam_params.cx) * zf / cam_params.fx;
		const float yc = (pt.y - cam_params.cy) * zf / cam_params.fy;
		const glm::vec3 cam_point(xc, yc, zf);
		const glm::vec3 obj_point = glm::transpose(rot) * (cam_point - trans);

		filtered_kp.push_back(keypoints[i]);
		filtered_desc.push_back(descriptors.row(static_cast<int>(i)));
		obj_points.emplace_back(obj_point.x, obj_point.y, obj_point.z);
	}
	if (filtered_desc.empty())
	{
		return;
	}

	KeypointTemplate temp;
	temp.keypoints = std::move(filtered_kp);
	temp.descriptors = filtered_desc;
	temp.objectPoints = std::move(obj_points);
	temp.bbox = bbox;
	temp.imageSize = gray.size();
	current_templates_.push_back(std::move(temp));
}

void KeypointDetector::pushBackTemplates(const std::string& class_id)
{
	class_ids_.push_back(class_id);
	templates_.push_back(current_templates_);
	current_templates_.clear();
}

void KeypointDetector::write(const std::string& filename) const
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	fs << "classes" << "[";
	for (size_t i = 0; i < class_ids_.size(); ++i)
	{
		fs << "{";
		fs << "class_id" << class_ids_[i];
		fs << "templates" << "[";
		for (const auto& temp : templates_[i])
		{
			fs << "{";
			fs << "bbox" << temp.bbox;
			fs << "image_width" << temp.imageSize.width;
			fs << "image_height" << temp.imageSize.height;
			fs << "keypoints" << temp.keypoints;
			fs << "descriptors" << temp.descriptors;
			fs << "object_points" << temp.objectPoints;
			fs << "}";
		}
		fs << "]";
		fs << "}";
	}
	fs << "]";
}

bool KeypointDetector::read(const std::string& filename)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		return false;
	}

	class_ids_.clear();
	templates_.clear();

	cv::FileNode classes = fs["classes"];
	for (auto&& cls : classes)
	{
		std::string class_id;
		cls["class_id"] >> class_id;
		class_ids_.push_back(class_id);

		std::vector<KeypointTemplate> class_templates;
		cv::FileNode templs = cls["templates"];
		for (auto&& t : templs)
		{
			KeypointTemplate temp;
			int width = 0;
			int height = 0;
			t["bbox"] >> temp.bbox;
			t["image_width"] >> width;
			t["image_height"] >> height;
			temp.imageSize = cv::Size(width, height);
			t["keypoints"] >> temp.keypoints;
			t["descriptors"] >> temp.descriptors;
			t["object_points"] >> temp.objectPoints;
			class_templates.push_back(std::move(temp));
		}
		templates_.push_back(std::move(class_templates));
	}
	return true;
}

bool KeypointDetector::detect(const cv::Mat& color, const std::string& class_id,
                              std::vector<cv::Rect>& out_boxes) const
{
	out_boxes.clear();
	if (templates_.empty())
	{
		return false;
	}

	auto it = std::find(class_ids_.begin(), class_ids_.end(), class_id);
	if (it == class_ids_.end())
	{
		return false;
	}
	const size_t class_index = std::distance(class_ids_.begin(), it);

	cv::Mat gray;
	if (color.channels() == 3)
	{
		cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
	}
	else
	{
		gray = color;
	}

	std::vector<cv::KeyPoint> kp_scene;
	cv::Mat desc_scene;
	orb_->detectAndCompute(gray, cv::Mat(), kp_scene, desc_scene);
	if (desc_scene.empty())
	{
		return false;
	}

	cv::BFMatcher matcher(cv::NORM_HAMMING);

	std::vector<Candidate> candidates;
	for (const auto& temp : templates_[class_index])
	{
		if (temp.descriptors.empty())
		{
			continue;
		}

		std::vector<std::vector<cv::DMatch>> knn;
		matcher.knnMatch(temp.descriptors, desc_scene, knn, 2);

		std::vector<cv::Point2f> pts_obj;
		std::vector<cv::Point2f> pts_scene;
		for (const auto& m : knn)
		{
			if (m.size() < 2)
			{
				continue;
			}
			if (m[0].distance < ratio_thresh_ * m[1].distance)
			{
				pts_obj.push_back(temp.keypoints[m[0].queryIdx].pt);
				pts_scene.push_back(kp_scene[m[0].trainIdx].pt);
			}
		}

		if (pts_obj.size() < 4)
		{
			continue;
		}

		std::vector<unsigned char> inlier_mask;
		cv::Mat H = cv::findHomography(pts_obj, pts_scene, cv::RANSAC, 3.0, inlier_mask);
		if (H.empty())
		{
			continue;
		}

		int inliers = 0;
		for (unsigned char inlier : inlier_mask)
		{
			inliers += inlier ? 1 : 0;
		}
		if (inliers < min_inliers_)
		{
			continue;
		}

		std::vector<cv::Point2f> obj_corners(4);
		obj_corners[0] = cv::Point2f((float)temp.bbox.x, (float)temp.bbox.y);
		obj_corners[1] = cv::Point2f((float)(temp.bbox.x + temp.bbox.width),
		                             (float)temp.bbox.y);
		obj_corners[2] = cv::Point2f((float)(temp.bbox.x + temp.bbox.width),
		                             (float)(temp.bbox.y + temp.bbox.height));
		obj_corners[3] = cv::Point2f((float)temp.bbox.x,
		                             (float)(temp.bbox.y + temp.bbox.height));

		std::vector<cv::Point2f> scene_corners;
		cv::perspectiveTransform(obj_corners, scene_corners, H);
		cv::Rect box = cv::boundingRect(scene_corners);
		box &= cv::Rect(0, 0, gray.cols, gray.rows);
		if (box.area() <= 0)
		{
			continue;
		}

		Candidate cand;
		cand.box = box;
		cand.inliers = inliers;
		candidates.push_back(cand);
	}

	if (candidates.empty())
	{
		return false;
	}

	std::sort(candidates.begin(), candidates.end(),
	          [](const Candidate& a, const Candidate& b)
	          {
		          return a.inliers > b.inliers;
	          });

	std::vector<Candidate> kept;
	for (const auto& cand : candidates)
	{
		bool overlaps = false;
		for (const auto& sel : kept)
		{
			if (iou(cand.box, sel.box) > nms_iou_thresh_)
			{
				overlaps = true;
				break;
			}
		}
		if (!overlaps)
		{
			kept.push_back(cand);
			if (kept.size() >= max_boxes_)
			{
				break;
			}
		}
	}

	for (const auto& sel : kept)
	{
		out_boxes.push_back(sel.box);
	}

	return !out_boxes.empty();
}

bool KeypointDetector::estimatePose(const cv::Mat& color, const std::string& class_id,
                                    const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,
                                    cv::Rect& out_box, cv::Mat& out_rvec, cv::Mat& out_tvec,
                                    int& out_inliers) const
{
	out_inliers = 0;
	out_box = cv::Rect();
	out_rvec.release();
	out_tvec.release();

	if (templates_.empty())
	{
		return false;
	}

	auto it = std::find(class_ids_.begin(), class_ids_.end(), class_id);
	if (it == class_ids_.end())
	{
		return false;
	}
	const size_t class_index = std::distance(class_ids_.begin(), it);

	cv::Mat gray;
	if (color.channels() == 3)
	{
		cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
	}
	else
	{
		gray = color;
	}

	std::vector<cv::KeyPoint> kp_scene;
	cv::Mat desc_scene;
	orb_->detectAndCompute(gray, cv::Mat(), kp_scene, desc_scene);
	if (desc_scene.empty())
	{
		return false;
	}

	cv::BFMatcher matcher(cv::NORM_HAMMING);

	for (const auto& temp : templates_[class_index])
	{
		if (temp.descriptors.empty() || temp.objectPoints.size() != temp.keypoints.size())
		{
			continue;
		}

		std::vector<std::vector<cv::DMatch>> knn;
		matcher.knnMatch(temp.descriptors, desc_scene, knn, 2);

		std::vector<cv::Point3f> obj_pts;
		std::vector<cv::Point2f> img_pts;
		for (const auto& m : knn)
		{
			if (m.size() < 2)
			{
				continue;
			}
			if (m[0].distance < ratio_thresh_ * m[1].distance)
			{
				const int q = m[0].queryIdx;
				const int t = m[0].trainIdx;
				obj_pts.push_back(temp.objectPoints[q]);
				img_pts.push_back(kp_scene[t].pt);
			}
		}

		if (obj_pts.size() < 6)
		{
			continue;
		}

		cv::Mat rvec, tvec;
		std::vector<int> inliers;
		bool ok = cv::solvePnPRansac(
			obj_pts, img_pts, camera_matrix, dist_coeffs, rvec, tvec, false,
			100, 8.0, 0.99, inliers, cv::SOLVEPNP_ITERATIVE);
		if (!ok)
		{
			continue;
		}

		const int num_inliers = static_cast<int>(inliers.size());
		if (num_inliers < min_inliers_ || num_inliers <= out_inliers)
		{
			continue;
		}

		std::vector<cv::Point2f> inlier_pts;
		inlier_pts.reserve(inliers.size());
		for (int idx : inliers)
		{
			inlier_pts.push_back(img_pts[idx]);
		}
		cv::Rect box = cv::boundingRect(inlier_pts);
		box &= cv::Rect(0, 0, gray.cols, gray.rows);
		if (box.area() <= 0)
		{
			continue;
		}

		out_inliers = num_inliers;
		out_box = box;
		out_rvec = rvec;
		out_tvec = tvec;
	}

	return out_inliers > 0;
}
