#include "depthai_bridge/SpatialDetectionConverter.hpp"

#include "depthai_bridge/depthaiUtility.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/quaternion.hpp> // OpenCV 4.7+ for cv::Quat
#include <cmath>

namespace dai {
namespace ros {

SpatialDetectionConverter::SpatialDetectionConverter(std::string frameName, int width, int height, std::shared_ptr<dai::Device> device, dai::CameraBoardSocket socket, bool normalized, bool getBaseDeviceTimestamp)
    : _frameName(frameName),
      _width(width),
      _height(height),
      _device(device),
      _socket(socket),
      _normalized(normalized),
      _steadyBaseTime(std::chrono::steady_clock::now()),
      _getBaseDeviceTimestamp(getBaseDeviceTimestamp) {
    _rosBaseTime = rclcpp::Clock().now();
    _ch = device->readCalibration();
}

SpatialDetectionConverter::~SpatialDetectionConverter() = default;

void SpatialDetectionConverter::updateRosBaseTime() {
    updateBaseTime(_steadyBaseTime, _rosBaseTime, _totalNsChange);
}

void SpatialDetectionConverter::toRosMsg(std::shared_ptr<dai::SpatialImgDetections> inNetData,
                                         std::deque<SpatialMessages::SpatialDetectionArray>& opDetectionMsgs) {
    if(_updateRosBaseTimeOnToRosMsg) {
        updateRosBaseTime();
    }
    std::chrono::_V2::steady_clock::time_point tstamp;
    if(_getBaseDeviceTimestamp)
        tstamp = inNetData->getTimestampDevice();
    else
        tstamp = inNetData->getTimestamp();
    SpatialMessages::SpatialDetectionArray opDetectionMsg;

    opDetectionMsg.header.stamp = getFrameTime(_rosBaseTime, _steadyBaseTime, tstamp);
    opDetectionMsg.header.frame_id = _frameName;
    opDetectionMsg.detections.resize(inNetData->detections.size());

    // TODO(Sachin): check if this works fine for normalized detection
    // publishing
    for(int i = 0; i < inNetData->detections.size(); ++i) {
        int xMin, yMin, xMax, yMax;
        if(_normalized) {
            xMin = inNetData->detections[i].xmin;
            yMin = inNetData->detections[i].ymin;
            xMax = inNetData->detections[i].xmax;
            yMax = inNetData->detections[i].ymax;
        } else {
            xMin = inNetData->detections[i].xmin * _width;
            yMin = inNetData->detections[i].ymin * _height;
            xMax = inNetData->detections[i].xmax * _width;
            yMax = inNetData->detections[i].ymax * _height;
        }

        float xSize = xMax - xMin;
        float ySize = yMax - yMin;
        float xCenter = xMin + xSize / 2;
        float yCenter = yMin + ySize / 2;
        opDetectionMsg.detections[i].results.resize(1);

        opDetectionMsg.detections[i].results[0].class_id = std::to_string(inNetData->detections[i].label);
        opDetectionMsg.detections[i].results[0].score = inNetData->detections[i].confidence;

        opDetectionMsg.detections[i].bbox.center.position.x = xCenter;
        opDetectionMsg.detections[i].bbox.center.position.y = yCenter;
        opDetectionMsg.detections[i].bbox.size_x = xSize;
        opDetectionMsg.detections[i].bbox.size_y = ySize;

        // converting mm to meters since per ros rep-103 lenght should always be in meters
        opDetectionMsg.detections[i].position.x = inNetData->detections[i].spatialCoordinates.x / 1000;
        opDetectionMsg.detections[i].position.y = inNetData->detections[i].spatialCoordinates.y / 1000;
        opDetectionMsg.detections[i].position.z = inNetData->detections[i].spatialCoordinates.z / 1000;
    }

    opDetectionMsgs.push_back(opDetectionMsg);
}

SpatialDetectionArrayPtr SpatialDetectionConverter::toRosMsgPtr(std::shared_ptr<dai::SpatialImgDetections> inNetData) {
    std::deque<SpatialMessages::SpatialDetectionArray> msgQueue;
    toRosMsg(inNetData, msgQueue);
    auto msg = msgQueue.front();
    SpatialDetectionArrayPtr ptr = std::make_shared<SpatialMessages::SpatialDetectionArray>(msg);
    return ptr;
}

void SpatialDetectionConverter::toRosVisionMsg(std::shared_ptr<dai::SpatialImgDetections> inNetData,
                                               std::deque<vision_msgs::msg::Detection3DArray>& opDetectionMsgs) {
    if(_updateRosBaseTimeOnToRosMsg) {
        updateRosBaseTime();
    }
    std::chrono::_V2::steady_clock::time_point tstamp;
    if(_getBaseDeviceTimestamp)
        tstamp = inNetData->getTimestampDevice();
    else
        tstamp = inNetData->getTimestamp();
    vision_msgs::msg::Detection3DArray opDetectionMsg;

    opDetectionMsg.header.stamp = getFrameTime(_rosBaseTime, _steadyBaseTime, tstamp);
    opDetectionMsg.header.frame_id = _frameName;
    opDetectionMsg.detections.resize(inNetData->detections.size());

    // TODO(Sachin): check if this works fine for normalized detection
    // publishing
    for(int i = 0; i < inNetData->detections.size(); ++i) {
        int xMin, yMin, xMax, yMax;
        if(_normalized) {
            xMin = inNetData->detections[i].xmin;
            yMin = inNetData->detections[i].ymin;
            xMax = inNetData->detections[i].xmax;
            yMax = inNetData->detections[i].ymax;
        } else {
            xMin = inNetData->detections[i].xmin * _width;
            yMin = inNetData->detections[i].ymin * _height;
            xMax = inNetData->detections[i].xmax * _width;
            yMax = inNetData->detections[i].ymax * _height;
        }

        float xSize = xMax - xMin;
        float ySize = yMax - yMin;
        float xCenter = xMin + xSize / 2;
        float yCenter = yMin + ySize / 2;
        opDetectionMsg.detections[i].results.resize(1);

        auto intrinsics = _ch.getCameraIntrinsics(_socket, _width, _height);

        auto p_z = inNetData->detections[i].spatialCoordinates.z / 1000; // to m
        auto fx = intrinsics[0][0];
        auto fy = intrinsics[1][1];
        auto p_x = inNetData->detections[i].spatialCoordinates.x / 1000;
        auto p_y = -1.0 * inNetData->detections[i].spatialCoordinates.y / 1000;

        opDetectionMsg.detections[i].results[0].hypothesis.class_id = std::to_string(inNetData->detections[i].label);
        opDetectionMsg.detections[i].results[0].hypothesis.score = inNetData->detections[i].confidence;
        opDetectionMsg.detections[i].bbox.center.position.x = xCenter;
        opDetectionMsg.detections[i].bbox.center.position.y = -1.0 * yCenter;
        auto s_x = xSize * (p_z / fx);
        auto s_y = ySize * (p_z / fy);
        opDetectionMsg.detections[i].bbox.size.x = s_x;
        opDetectionMsg.detections[i].bbox.size.y = s_y;
        opDetectionMsg.detections[i].bbox.size.z = std::min(s_x, s_y) / 2.0;

        opDetectionMsg.detections[i].results[0].pose.pose.position.x = p_x;
        opDetectionMsg.detections[i].results[0].pose.pose.position.y = p_y;
        opDetectionMsg.detections[i].results[0].pose.pose.position.z = p_z;

        // make BBs face the camera
        cv::Vec3d forward(-1.0 * p_x,
                          -1.0 * p_y,
                          -1.0 * p_z
                         );
        forward /= cv::norm(forward);

        cv::Vec3d worldUp(0, 1, 0);
        if (std::abs(forward.dot(worldUp)) > 0.999) // nearly parallel
            worldUp = cv::Vec3d(0, 0, 1);

        cv::Vec3d right = worldUp.cross(forward);
        right /= cv::norm(right);
        cv::Vec3d up = forward.cross(right);
        up /= cv::norm(up);

        cv::Matx33d rot(
            right[0],   up[0],   forward[0],
            right[1],   up[1],   forward[1],
            right[2],   up[2],   forward[2]
        );

        auto q = cv::Quatd::createFromRotMat(rot);
        opDetectionMsg.detections[i].results[0].pose.pose.orientation.x = q.x;
        opDetectionMsg.detections[i].results[0].pose.pose.orientation.y = q.y;
        opDetectionMsg.detections[i].results[0].pose.pose.orientation.z = q.z;
        opDetectionMsg.detections[i].results[0].pose.pose.orientation.w = q.w;
    }

    opDetectionMsgs.push_back(opDetectionMsg);
}

}  // namespace ros
}  // namespace dai
