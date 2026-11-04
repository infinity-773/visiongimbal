#ifndef SUPER_STRUCTURE_H_
#define SUPER_STRUCTURE_H_

#include "Serial.h"
#include "TargetContainer.h"
#include <algorithm>
#include <cmath>
#include <vector>


struct axisData
{
    double yaw;
    double pitch;

    axisData(double yawValue = 0.0, double pitchValue = 0.0)
        : yaw(yawValue), pitch(pitchValue)
    {}
};

struct OutputData
{
    axisData axis;
    bool fire = false;
    bool useABSposition = true;
};



 static double ZERO_PITCH = 0.0;
 static double ZERO_YAW = 0.0;
 static int STAY_TIME = 1000; // ms
 static double PITCH_BIAS = 0.0; //g补偿
 static double GAIN = 0.07;
 static axisData CAMERA_BIAS(0.0, 0.0); //相机偏差

enum class State
{
    aiming,
    aimed,
    zeroing,
    zeroed,
    lost
};

class SuperStructure
{
public:
    SuperStructure()
        : state_(State::zeroed),
          targetLostAt_(0)
    {
        output_.axis.yaw = ZERO_YAW;
        output_.axis.pitch = ZERO_PITCH;
        output_.fire = false;
        output_.useABSposition = true;
        boxes_ = BoxContainer{};
    }

    void updateBoxes(const BoxContainer& boxes)
    {
        boxes_ = boxes;
    }

    void periodic(const TRoMaC::GimbalFeedback& feedback, int now)
    {   // 更新反馈数据
        feedbackData_.yaw = feedback.yaw_angle;
        feedbackData_.pitch = feedback.pitch_angle;

        // 仅在恰好检测到一个框时取其中心作为目标。
        const bool targetValid = boxes_.size() == 1;
        const cv::Point2d target = targetValid ? boxes_.getCenter(0) : cv::Point2d();

        switch (state_)
        {
        case State::aiming://定义 ： 目标存在但未对准

            if (targetValid)//目标一直存在 -> 持续跟随
            {
                output_.axis = getAxissetpoint(target);
                if (std::fabs(feedbackData_.yaw - output_.axis.yaw) < 0.2 &&
                    std::fabs(feedbackData_.pitch - output_.axis.pitch) < 0.2)
                {
                    state_ = State::aimed;
                }
            }else{
                state_ = State::lost;
                targetLostAt_ = now; //记录丢失目标的时间,在这里定义可以减少一帧误差
            }
            break;

        case State::aimed://定义 ： 目标存在且对准

            if (!targetValid)
            {
                state_ = State::lost;
                targetLostAt_ = now;
            }
            else
            {
                output_.axis = getAxissetpoint(target);
                if (std::fabs(feedbackData_.yaw - output_.axis.yaw) >= 0.2 ||
                    std::fabs(feedbackData_.pitch - output_.axis.pitch) >= 0.2)
                {
                    state_ = State::aiming;
                }
            }
            break;

        case State::zeroed://定义 ： 回正完成
            if (targetValid)
            {
                state_ = State::aiming;
            }
            break;

        case State::zeroing://定义 ： 回正中

            if (targetValid) //如果有目标立刻打断回正
            {
                state_ = State::aiming;
                break;
            }else if (
                std::fabs(feedbackData_.yaw - ZERO_YAW) < 0.5 &&
                std::fabs(feedbackData_.pitch - ZERO_PITCH) < 0.5)
            {
                state_ = State::zeroed;
                break;
            }
            output_.axis.yaw = ZERO_YAW;
            output_.axis.pitch = ZERO_PITCH;
            break;

        case State::lost://定义 ： 目标丢失 ，一小段时间无目标后zreo，zero流程只有此一个入口
            if (!targetValid && (now - targetLostAt_) > STAY_TIME)
            {
                state_ = State::zeroing;
            }
            else if (targetValid)//有目标 -> 跟上
            {
                state_ = State::aiming;
            }
            break;

        default:
            break;
        }
    }

    const OutputData& getoutput() const
    {
        return output_;
    }

    const char* getstateName() const
    {
        switch (state_)
        {
        case State::aiming: return "aiming";
        case State::aimed: return "aimed";
        case State::zeroing: return "zeroing";
        case State::zeroed: return "zeroed";
        case State::lost: return "lost";
        }
        return "unknown";
    }

    bool isAimed() const
    {
        return state_ == State::aimed && boxes_.size() == 1;
    }

private:
    State state_;
    OutputData output_;
    axisData feedbackData_;
    int targetLostAt_;
    BoxContainer boxes_;

    axisData getAxissetpoint(const cv::Point2d& target)
    {
        constexpr double aimU = 640.0; //图像中心
        constexpr double aimV = 360.0; //图像中心
        constexpr double focalPx = 700.0; //相机焦距像素值

        const double yawError = -pixelToAngle(target.x - aimU, focalPx);
        const double pitchError = pixelToAngle(target.y - aimV, focalPx);

        return axisData(
            feedbackData_.yaw + GAIN * yawError + CAMERA_BIAS.yaw,
            feedbackData_.pitch + GAIN * pitchError + CAMERA_BIAS.pitch + PITCH_BIAS);
    }

    // 像素差转角度 （单位：度）
    static double pixelToAngle(double pixels, double focalPx)
    {
     return std::atan(pixels / focalPx) * 180.0 / CV_PI;
    }
};

#endif // SUPER_STRUCTURE_H_
