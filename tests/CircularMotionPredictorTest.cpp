#include "CircularMotionPredictor.h"

#include <cmath>
#include <iostream>

static cv::Point2d projectToCamera(const cv::Point2d& point, double yaw, double pitch)
{
    const double yawRadians = -yaw * CV_PI / 180.0;
    const double pitchRadians = -pitch * CV_PI / 180.0;
    const double yawCos = std::cos(yawRadians);
    const double yawSin = std::sin(yawRadians);
    const double pitchCos = std::cos(pitchRadians);
    const double pitchSin = std::sin(pitchRadians);
    const cv::Vec3d world((point.x - 640.0) / 1300.0,
                          (point.y - 360.0) / 1300.0, 1.0);
    const cv::Vec3d yawAligned(yawCos * world[0] - yawSin * world[2],
                               world[1], yawSin * world[0] + yawCos * world[2]);
    const cv::Vec3d camera(yawAligned[0],
                           pitchCos * yawAligned[1] + pitchSin * yawAligned[2],
                           -pitchSin * yawAligned[1] + pitchCos * yawAligned[2]);
    return cv::Point2d(640.0 + 1300.0 * camera[0] / camera[2],
                       360.0 + 1300.0 * camera[1] / camera[2]);
}

static bool runCase(bool movingCamera)
{
    CircularMotionPredictor predictor(1300.0, 640.0, 360.0, 1.0, 1.0);
    CircularPrediction result;
    const double predictionSeconds = 0.15;
    for (int index = 0; index < 90; ++index)
    {
        const double time = index / 30.0;
        const double yaw = movingCamera ? 4.0 * std::sin(time) : 0.0;
        const double pitch = movingCamera ? 3.0 * std::sin(time * 0.7) : 0.0;
        const cv::Point2d point(700.0 + 100.0 * std::cos(time),
                                330.0 + 100.0 * std::sin(time));
        result = predictor.update(projectToCamera(point, yaw, pitch), yaw, pitch,
                                  time, predictionSeconds);
        if (index < 17 && result.valid)
        {
            return false;
        }
    }
    const double time = 89.0 / 30.0;
    const double yaw = movingCamera ? 4.0 * std::sin(time) : 0.0;
    const double pitch = movingCamera ? 3.0 * std::sin(time * 0.7) : 0.0;
    const cv::Point2d future(700.0 + 100.0 * std::cos(time + predictionSeconds),
                             330.0 + 100.0 * std::sin(time + predictionSeconds));
    if (!result.valid || std::fabs(result.angularVelocity - 1.0) > 0.01 ||
        cv::norm(result.predicted - projectToCamera(future, yaw, pitch)) > 1.0)
    {
        return false;
    }
    return !predictor.update(cv::Point2d(700.0, 330.0), yaw, pitch,
                             time + 1.0, predictionSeconds).valid;
}

int main()
{
    if (!runCase(false) || !runCase(true))
    {
        std::cerr << "Circular motion prediction test failed\n";
        return 1;
    }
    std::cout << "Circular motion prediction test passed\n";
    return 0;
}