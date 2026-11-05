#include "PIDController.h"

#include "MathUtils.h"

#include <cmath>
#include <stdexcept>

PIDController::PIDController(const PIDConfig& config)
    : config_(config),
      integral_(0.0),
      previousError_(0.0),
      previousPreviousError_(0.0),
      previousOutput_(0.0),
      hasPreviousError_(false),
      hasPreviousPreviousError_(false)
{
    if (!std::isfinite(config_.kp) || !std::isfinite(config_.ki) ||
        !std::isfinite(config_.kd) || !std::isfinite(config_.integralMin) ||
        !std::isfinite(config_.integralMax) || !std::isfinite(config_.outputMin) ||
        !std::isfinite(config_.outputMax) || config_.integralMin > config_.integralMax ||
        config_.outputMin > config_.outputMax)
    {
        throw std::invalid_argument("Invalid PID configuration");
    }
}

double PIDController::update(double error, double dtSeconds, PIDMode mode)
{
    if (!std::isfinite(error) || !std::isfinite(dtSeconds) || dtSeconds <= 0.0)
    {
        throw std::invalid_argument("PID update requires a finite error and positive dt");
    }

    return mode == PIDMode::Position
        ? updatePosition(error, dtSeconds)
        : updateIncremental(error, dtSeconds);
}

void PIDController::reset()
{
    integral_ = 0.0;
    previousError_ = 0.0;
    previousPreviousError_ = 0.0;
    previousOutput_ = 0.0;
    hasPreviousError_ = false;
    hasPreviousPreviousError_ = false;
}

double PIDController::updatePosition(double error, double dtSeconds)
{
    const double derivative = hasPreviousError_
        ? (error - previousError_) / dtSeconds
        : 0.0;
    const double candidateIntegral = clampValue(
        integral_ + error * dtSeconds, config_.integralMin, config_.integralMax);
    const double candidateOutput =
        config_.kp * error + config_.ki * candidateIntegral + config_.kd * derivative;

    if ((candidateOutput >= config_.outputMin && candidateOutput <= config_.outputMax) ||
        (candidateOutput > config_.outputMax && error < 0.0) ||
        (candidateOutput < config_.outputMin && error > 0.0))
    {
        integral_ = candidateIntegral;
    }

    const double output = clampValue(
        config_.kp * error + config_.ki * integral_ + config_.kd * derivative,
        config_.outputMin, config_.outputMax);
    previousError_ = error;
    hasPreviousError_ = true;
    return output;
}

double PIDController::updateIncremental(double error, double dtSeconds)
{
    const double previousError = hasPreviousError_ ? previousError_ : error;
    const double previousPreviousError =
        hasPreviousPreviousError_ ? previousPreviousError_ : previousError;
    const double deltaOutput =
        config_.kp * (error - previousError) +
        config_.ki * error * dtSeconds +
        config_.kd * (error - 2.0 * previousError + previousPreviousError) / dtSeconds;

    const double limitedOutput = clampValue(
        previousOutput_ + deltaOutput, config_.outputMin, config_.outputMax);
    const double appliedDelta = limitedOutput - previousOutput_;
    previousOutput_ = limitedOutput;
    previousPreviousError_ = previousError;
    hasPreviousPreviousError_ = hasPreviousError_;
    previousError_ = error;
    hasPreviousError_ = true;
    return appliedDelta;
}
