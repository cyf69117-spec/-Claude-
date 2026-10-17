#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习).
 *            The code ADAPTS the corresponding hardware circuit board(智能汽车-ICAR),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file motion.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 运动控制器
 * @version 0.1
 * @date 2025-07-14
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <cmath>
#include <numeric>
#include "utils/tools.hpp"
#include "utils/params.hpp"

using namespace std;

class Motion
{

public:
    /**
     * @brief 姿态PD控制器
     *
     * @param center 智能车控制中心
     */
    void poseControl(shared_ptr<Params> &params)
    {
        const float center = static_cast<float>(params->ctrl.center);
        const float errorRaw = center - COLSIMAGE / 2.0f;
        const float errorStepMax = COLSIMAGE / 10.0f; // 每帧最大步进32像素

        // 误差渐变只用于P项，errorLast仅保存上一帧整形后的误差。
        // 直接限制每帧最大步进，避免旧16像素触发逻辑导致误差反向跳变。
        float errorUsed = errorRaw;
        const float errorStep = errorRaw - errorLast;
        if (errorStep > errorStepMax)
            errorUsed = errorLast + errorStepMax;
        else if (errorStep < -errorStepMax)
            errorUsed = errorLast - errorStepMax;

        errorLast = errorUsed;

        // 动态P增益保持原公式，使用整形误差，runP1和runP2只读。
        params->config.turnP = std::abs(errorUsed) * params->config.runP2 + params->config.runP1;

        // 死区根据原始误差判断，仅将P项输出置零，保留D项。
        // 进入死区后仍更新整形历史，保证后续P项变化连续。
        const float pTerm = std::abs(errorRaw) <= POSE_DEADZONE
                                ? 0.0f
                                : errorUsed * params->config.turnP;

        // 微分先行：D项使用原始center差分，与P项整形完全解耦。
        // 目标中心固定，测量差分与原始误差差分同号，保持原D项符号。
        float dRaw = 0.0f;
        if (!poseHistoryValid || params->ctrl.stop)
        {
            // 首次调用、复位后和停车期间不产生微分冲击。
            // 停车时center可能变化或仍为旧值，显式清除滤波历史。
            dFilt = 0.0f;
        }
        else
        {
            dRaw = center - centerLast;
        }

        // 一阶IIR低通抑制单帧毛刺；center恒定时D项逐帧衰减至0。
        dFilt += POSE_D_ALPHA * (dRaw - dFilt);
        centerLast = center; // 始终保存原始测量，不能保存整形误差
        poseHistoryValid = !params->ctrl.stop; // 停车恢复首帧重新建立基准

        // 保持30fps下像素/帧的差分尺度，不除以帧周期。
        // 原整形时D项参考幅值约为32 * 3.5 = 112 PWM，新D项为dFilt * turnD。
        // 真实测量突变时D项可能超过该参考值，最终由总PWM限幅保护。
        // turnD建议沿用3.5起步；上车先设turnD=0验证P项行为与原车手感，
        // 再从2.0起逐步增加D，检查直线抖动及弯道响应。
        const float dTerm = dFilt * params->config.turnD;

        // 误差及历史计算使用float，只在最终PWM偏移处转为int。
        // 先在有符号int域内限幅，避免负数提前转换uint16_t后回绕。
        int pwmDiff = static_cast<int>(pTerm + dTerm);
        const int pwmDiffMin = PWMSERVOMIN - PWMSERVOMID;
        const int pwmDiffMax = PWMSERVOMAX - PWMSERVOMID;
        if (pwmDiff > pwmDiffMax)
            pwmDiff = pwmDiffMax;
        else if (pwmDiff < pwmDiffMin)
            pwmDiff = pwmDiffMin;

        // 有符号加法结果已在舵机有效范围内，最后转换为uint16_t。
        params->ctrl.servo = static_cast<uint16_t>(PWMSERVOMID + pwmDiff);
    }

    void resetPose()
    {
        // 模式切换时清除全部历史，由主循环在停车恢复和倒车切换时调用。
        errorLast = 0.0f;
        centerLast = 0.0f;
        dFilt = 0.0f;
        // 清零后的centerLast不是有效测量，下一帧先建立微分基准。
        poseHistoryValid = false;
    }

    /**
     * @brief 变加速控制
     *
     * @param params
     */
    void speedControl(shared_ptr<Params> &params)
    {
        if (params->ctrl.stop || params->ctrl.obstacleStop) // 路线停车或障碍物停车，最高优先级
        {
            params->ctrl.speed = 0.0;
            return;
        }
        if (params->ctrl.back) // 倒车(停车场)
        {
            params->ctrl.speed = -params->config.velPark;
            return;
        }
        if (params->mode == FsmMode::STOP) // 停车区速度
        {
            params->ctrl.speed = params->config.velStop;
        }
        else if (params->mode == FsmMode::PARK) // 停车场速度
        {
            params->ctrl.speed = params->config.velPark;
        }
        else if (params->mode == FsmMode::CROSS) // 斑马线速度
        {
            params->ctrl.speed = params->config.velCross;
        }
        else if (params->mode == FsmMode::STATION) // 停靠站速度
        {
            params->ctrl.speed = params->config.velSlow;
        }
        else if (params->mode == FsmMode::BUSY || params->busyZone) // 施工区自动通行速度
        {
            // 施工区遇到已确认障碍物时，避障减速优先，且不高于原施工区速度。
            params->ctrl.speed = params->ctrl.obstacleSlow
                                     ? std::min(params->config.velBusy, params->config.velSlow)
                                     : params->config.velBusy;
        }
        else if (params->mode == FsmMode::CURVE) // 连续弯道速度
        {
            params->ctrl.speed = params->config.velCurve;
        }
        else if (params->mode == FsmMode::YFORK) // Y型岔路口速度
        {
            params->ctrl.speed = params->config.velYfork;
        }
        else if (params->ctrl.slow) // 慢行区或避障独立减速
        {
            params->ctrl.speed = params->config.velSlow;
        }
        else if (params->ctrl.countAcc < 50)
        {
            params->ctrl.countAcc++;
            params->ctrl.speed = params->config.velCross;
        }

        else
        {
            int line = params->ctrl.lineArea; // 动态速度，返回点越高，速度越大，线性变化

            float upper = ROWSIMAGE * 0.35; // 84
            line = std::max(line - 20, 0);
            if (line > upper)
                line = upper;

            params->ctrl.speed = params->config.velLow +
                                 std::pow(float(upper - line) / (upper), 3) * (params->config.velHigh - params->config.velLow);
            if (params->ctrl.speed > params->config.velHigh)
                params->ctrl.speed = params->config.velHigh;

            if (params->alertDecelCount > 0)
                params->ctrl.speed = std::max(0.0f, params->ctrl.speed - 0.1f);
        }
        // 所有前进模式共用避障限速，施工区/停车场等场景不得覆盖。
        if (params->ctrl.obstacleSlow)
            params->ctrl.speed = std::max(0.0f, std::min(params->ctrl.speed, params->config.velSlow));
    }

    /**
     * @brief 显示赛道线识别结果
     *
     * @param img 需要叠加显示的图像
     */
    void drawImage(shared_ptr<Params> &params, Mat &img)
    {
        std::string str = "Vel: " + doble2String(params->ctrl.speed, 2);
        putText(img, str, Point(COLSIMAGE - 100, 120), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 速度
    }

    /**
     * @brief 车辆冲出赛道检测（保护车辆）
     *
     * @param track
     * @return true
     * @return false
     */
    void outlineCheck(shared_ptr<Params> &params)
    {
        // 安全停车锁存后只能由主循环发送停车帧并退出，不能被FSM解除。
        if (params->safetyStop)
        {
            params->ctrl.stop = true;
            return;
        }
        if (params->ctrl.outlineCooldown > 0)
        {
            --params->ctrl.outlineCooldown; // 30fps下90帧约3秒，每帧只递减一次
            countRes = 0;
            return;
        }
        if (params->ctrl.parking || params->config.debug)
        {
            countRes = 0;
            return;
        }
        if (params->track->pointsEdgeLeft.size() < 30 &&
            params->track->pointsEdgeRight.size() < 30)
        {
            if (++countRes > 20) // 连续21帧丢失双边线，沿用原有效检测阈值
            {
                params->safetyStop = true;
                params->shutdownRequested = true;
                params->ctrl.stop = true;
                params->ctrl.speed = 0.0f;
                params->ctrl.servo = PWMSERVOMID;
                resetPose();
            }
        }
        else
            countRes = 0; // 有效边线恢复即清零，避免间歇丢线跨路段累计
    }

private:
    static constexpr float POSE_D_ALPHA = 0.4f; // 可调：越小越平滑但响应越迟钝
    static constexpr float POSE_DEADZONE = 2.0f; // 可调：P项死区阈值，单位为像素

    float errorLast = 0.0f;  // 上一帧整形后的P项误差
    float centerLast = 0.0f; // 上一帧原始控制中心
    float dFilt = 0.0f;      // 测量差分的一阶低通输出
    bool poseHistoryValid = false; // 是否已有有效的微分测量基准

    int countRes = 0;
};
