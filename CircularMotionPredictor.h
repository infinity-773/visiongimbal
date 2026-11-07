#ifndef CIRCULAR_MOTION_PREDICTOR_H_
#define CIRCULAR_MOTION_PREDICTOR_H_

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

struct CircularPrediction
{
    bool valid = false;
    cv::Point2d center;
    double radius = 0.0;
    double phase = 0.0;
    double angularVelocity = 0.0;
    cv::Point2d predicted;
    std::vector<cv::Point2d> trajectory;
};

class CircularMotionPredictor
{
public:
    CircularMotionPredictor(double focalLength, double centerX, double centerY,
                            double yawSign, double pitchSign)
        : focalLength_(focalLength), centerX_(centerX), centerY_(centerY),
          yawSign_(yawSign), pitchSign_(pitchSign), hasReference_(false)
    {}

    void reset()
    {
        samples_.clear();
        hasReference_ = false;
    }

    CircularPrediction update(const cv::Point2d& pixel, double yaw, double pitch,
                              double time, double predictionSeconds)
    {
        CircularPrediction result;
        if (!std::isfinite(pixel.x) || !std::isfinite(pixel.y) ||
            !std::isfinite(yaw) || !std::isfinite(pitch) || !std::isfinite(time) ||
            !std::isfinite(predictionSeconds) || predictionSeconds < 0.0 ||
            !std::isfinite(focalLength_) || focalLength_ <= 0.0)
        {
            reset();
            return result;
        }

        if (!samples_.empty() &&
            (time <= samples_.back().time || time - samples_.back().time > 0.3))
        {
            reset();
        }
        const cv::Matx33d currentRotation = rotation(yaw, pitch);
        if (!hasReference_)
        {
            referenceRotation_ = currentRotation;
            hasReference_ = true;
        }

        cv::Point2d referencePoint;
        if (!transform(pixel, referenceRotation_.t() * currentRotation, referencePoint))
        {
            reset();
            return result;
        }
        samples_.push_back(Sample{referencePoint, time});
        while (samples_.size() > 120 || time - samples_.front().time > 4.0)
        {
            samples_.pop_front();
        }
        if (samples_.size() < 18)
        {
            return result;
        }

        cv::Point2d mean;
        for (const Sample& sample : samples_)
        {
            mean += sample.point;
        }
        mean *= 1.0 / samples_.size();
        cv::Mat coefficients(static_cast<int>(samples_.size()), 3, CV_64F);
        cv::Mat values(static_cast<int>(samples_.size()), 1, CV_64F);
        for (std::size_t index = 0; index < samples_.size(); ++index)
        {
            const cv::Point2d point = samples_[index].point - mean;
            coefficients.at<double>(static_cast<int>(index), 0) = 2.0 * point.x;
            coefficients.at<double>(static_cast<int>(index), 1) = 2.0 * point.y;
            coefficients.at<double>(static_cast<int>(index), 2) = 1.0;
            values.at<double>(static_cast<int>(index)) = point.dot(point);
        }
        cv::Mat solution;
        if (!cv::solve(coefficients, values, solution, cv::DECOMP_SVD))
        {
            return result;
        }
        const cv::Point2d offset(solution.at<double>(0), solution.at<double>(1));
        const double radiusSquared = offset.dot(offset) + solution.at<double>(2);
        if (!std::isfinite(radiusSquared) || radiusSquared < 100.0 ||
            radiusSquared > 4000000.0)
        {
            return result;
        }
        const cv::Point2d center = mean + offset;
        const double radius = std::sqrt(radiusSquared);
        std::vector<double> phases;
        std::vector<double> sortedPhases;
        double residualSquared = 0.0;
        for (const Sample& sample : samples_)
        {
            const cv::Point2d delta = sample.point - center;
            const double residual = cv::norm(delta) - radius;
            residualSquared += residual * residual;
            double phase = std::atan2(delta.y, delta.x);
            sortedPhases.push_back(phase);
            if (!phases.empty())
            {
                phase = phases.back() + std::remainder(phase - phases.back(), 2.0 * CV_PI);
            }
            phases.push_back(phase);
        }
        if (std::sqrt(residualSquared / samples_.size()) > std::max(3.0, radius * 0.04))
        {
            return result;
        }
        std::sort(sortedPhases.begin(), sortedPhases.end());
        double maxGap = sortedPhases.front() + 2.0 * CV_PI - sortedPhases.back();
        for (std::size_t index = 1; index < sortedPhases.size(); ++index)
        {
            maxGap = std::max(maxGap, sortedPhases[index] - sortedPhases[index - 1]);
        }
        if (2.0 * CV_PI - maxGap < 1.2)
        {
            return result;
        }

        double meanTime = 0.0;
        double meanPhase = 0.0;
        for (std::size_t index = 0; index < samples_.size(); ++index)
        {
            meanTime += samples_[index].time - time;
            meanPhase += phases[index];
        }
        meanTime /= samples_.size();
        meanPhase /= samples_.size();
        double covariance = 0.0;
        double timeVariance = 0.0;
        for (std::size_t index = 0; index < samples_.size(); ++index)
        {
            const double deltaTime = samples_[index].time - time - meanTime;
            covariance += deltaTime * (phases[index] - meanPhase);
            timeVariance += deltaTime * deltaTime;
        }
        if (timeVariance < 1e-6)
        {
            return result;
        }
        const double velocity = covariance / timeVariance;
        const double predictedPhase = phases.back() + velocity * predictionSeconds;
        const cv::Point2d future(center.x + radius * std::cos(predictedPhase),
                                 center.y + radius * std::sin(predictedPhase));
        const cv::Matx33d toCurrent = currentRotation.t() * referenceRotation_;
        if (!transform(center, toCurrent, result.center) ||
            !transform(future, toCurrent, result.predicted))
        {
            return CircularPrediction();
        }
        for (int index = 0; index <= 96; ++index)
        {
            const double angle = 2.0 * CV_PI * index / 96.0;
            const cv::Point2d point(center.x + radius * std::cos(angle),
                                    center.y + radius * std::sin(angle));
            cv::Point2d projected;
            if (!transform(point, toCurrent, projected))
            {
                return CircularPrediction();
            }
            result.trajectory.push_back(projected);
        }
        result.valid = true;
        result.radius = radius;
        result.phase = phases.back();
        result.angularVelocity = velocity;
        return result;
    }

private:
    struct Sample
    {
        cv::Point2d point;
        double time;
    };

    cv::Matx33d rotation(double yaw, double pitch) const
    {
        const double yawRadians = -yawSign_ * yaw * CV_PI / 180.0;
        const double pitchRadians = -pitchSign_ * pitch * CV_PI / 180.0;
        const double yawCos = std::cos(yawRadians);
        const double yawSin = std::sin(yawRadians);
        const double pitchCos = std::cos(pitchRadians);
        const double pitchSin = std::sin(pitchRadians);
        const cv::Matx33d yawRotation(yawCos, 0.0, yawSin,
                                      0.0, 1.0, 0.0,
                                      -yawSin, 0.0, yawCos);
        const cv::Matx33d pitchRotation(1.0, 0.0, 0.0,
                                        0.0, pitchCos, -pitchSin,
                                        0.0, pitchSin, pitchCos);
        return yawRotation * pitchRotation;
    }

    bool transform(const cv::Point2d& pixel, const cv::Matx33d& rotation,
                   cv::Point2d& result) const
    {
        const cv::Vec3d ray((pixel.x - centerX_) / focalLength_,
                            (pixel.y - centerY_) / focalLength_, 1.0);
        const cv::Vec3d transformed = rotation * ray;
        if (transformed[2] <= 1e-6)
        {
            return false;
        }
        result = cv::Point2d(centerX_ + focalLength_ * transformed[0] / transformed[2],
                             centerY_ + focalLength_ * transformed[1] / transformed[2]);
        return std::isfinite(result.x) && std::isfinite(result.y) &&
               std::fabs(result.x) < 1e6 && std::fabs(result.y) < 1e6;
    }

    double focalLength_;
    double centerX_;
    double centerY_;
    double yawSign_;
    double pitchSign_;
    bool hasReference_;
    cv::Matx33d referenceRotation_;
    std::deque<Sample> samples_;
};

#endif