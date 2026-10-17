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
 * @file station.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 停靠站停车控制
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/station.hpp"

FsmStation::FsmStation(std::shared_ptr<Params> par)
    : FSMState(FsmMode::STATION, par)
{
}

FsmStation::~FsmStation()
{
}

FsmMode FsmStation::getMode()
{
    if (params->config.currentLapConfig && params->config.currentLapConfig->busy &&
        (!params->busyZone || params->busyGuiding || !params->config.currentLapConfig->busyStopEnable ||
         params->config.currentLapConfig->busyStopPoint == 0))
        return FsmMode::NORMAL;
    if (step != Step::NONE || pressTimer > 0)
        return FsmMode::STATION;
    else
        return FsmMode::NORMAL;
}

void FsmStation::run(Mat &img)
{
    std::uint64_t revision = 0;
    const auto results = params->resultsSnapshot(&revision); // 帧内快照，AI发布版本用于框消失确认
    const bool fresh = revision != lastResultsRevision;
    lastResultsRevision = revision;
    if (!params->config.currentLapConfig || !params->config.station ||
        !params->config.currentLapConfig->station)
        return;
    const bool construction = params->config.currentLapConfig->busy;
    if (construction && (!params->busyZone || params->busyGuiding ||
        !params->config.currentLapConfig->busyStopEnable ||
        params->config.currentLapConfig->busyStopPoint == 0 || params->stationStopCompleted))
        return; // 在施工区内部按命令停靠，入口/出口转向及返回主赛道时不停车。

    if (construction && fresh)
    {
        bool stationVisible = false;
        for (const auto &r : results)
            stationVisible = stationVisible || r.type == LABEL_STATION;
        stationLostCount = stationVisible ? 0 : stationLostCount + 1;
        if (stationLostCount >= 3)
            stationBoxCounted = false; // 前一个框离开后才能计下一个，不能永远跳过第二框。
    }

    countInit++;
    if (countInit > 999)
        countInit = 999;
    else if (countInit < 60) // 发车屏蔽
        return;

    switch (step)
    {
    case Step::NONE:
    {
        // 停车后冷却期内不检测
        if (cooldown > 0)
        {
            cooldown--;
            break;
        }

        if (pressTimer > 0)
        {
            pressTimer++;
            // 保留原停靠延时：施工区中间40帧、出口20帧、左岔路33帧、其他19帧。
            int pressThreshold = construction
                                     ? (params->config.currentLapConfig->busyStopPoint == 1 ? 40 : 20)
                                     : 19;
            if (params->yforkBranch == 1)
                pressThreshold = 33;
            if (pressTimer > pressThreshold)
            {
                printf("[Station] Pressed +%.1fs, stopping...\n", pressThreshold / 30.0f);
                setStep(Step::STOP);
            }
            break;
        }

        // 左岔路：引导结束见框开始计时（要求框到底部0.5以下，给足时间完成左转）
        if (params->yforkBranch == 1)
        {
            // 引导期间不检测
            if (params->yforkGuiding)
                break;

            for (int i = 0; i < results.size(); i++)
            {
                if (results[i].type == LABEL_STATION)
                {
                    int boxCx = results[i].x + results[i].width / 2;
                    int boxBottom = results[i].y + results[i].height;
                    if (boxCx < COLSIMAGE / 2 && boxBottom > ROWSIMAGE * 0.5)
                    {
                        params->stationStarted = true;
                        pressTimer = 1;
                        printf("[Station] Left branch, seen, 0.3s stop\n");
                        break;
                    }
                }
            }
        }
        else
        {
            // yfork引导期间不检测station框，避免干扰岔路导航
            if (params->yforkGuiding)
                break;

            // 非左岔路：框到底部才检测
            for (int i = 0; i < results.size(); i++)
            {
                if (results[i].type == LABEL_STATION)
                {
                    int boxBottom = results[i].y + results[i].height;
                    const int threshold = construction && stationBoxCounter == 0
                                              ? static_cast<int>(ROWSIMAGE * 0.5)
                                              : ROWSIMAGE - 10;
                    if (boxBottom > threshold)
                    {
                        if (construction)
                        {
                            if (stationBoxCounted)
                                break;
                            const int target = params->config.currentLapConfig->busyStopPoint;
                            if (stationBoxCounter < target - 1)
                            {
                                ++stationBoxCounter;
                                stationBoxCounted = true;
                                break; // 按命令跳过前面的框，不能一进施工区就在任意框停车。
                            }
                        }
                        params->stationStarted = true;
                        pressTimer = 1;
                        printf("[Station] Pressed\n");
                        break;
                    }
                }
            }
        }
        break;
    }

    case Step::STOP:
    {
        params->ctrl.stop = true;
        stopCounter++;
        printf("[Station] Stop %d/30\n", stopCounter);
        if (stopCounter > 30) // 停车约1秒
        {
            printf("[Station] Stop end, resume\n");
            params->stationStopCompleted = true; // 通知yfork边线突变可以退出了
            setStep(Step::NONE);
            cooldown = construction ? 6 : 150; // 沿用原施工区与普通停靠冷却
        }
        break;
    }
    }
}

void FsmStation::show(Mat &img)
{
    if (params->mode != FsmMode::STATION)
        return;

    putText(img, "[10] Station", Point(COLSIMAGE / 2 - 50, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);

    switch (step)
    {
    case Step::STOP:
        putText(img, "[10] Station - STOP", Point(100, 50),
                cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5);
        break;
    }
}

void FsmStation::setStep(Step st)
{
    step = st;
    stopCounter = 0;
    pressTimer = 0;
    params->ctrl.stop = false;
}

void FsmStation::resetLap()
{
    setStep(Step::NONE);
    countInit = 0;
    cooldown = 0;
    params->stationStopCompleted = false;
    params->stationStarted = false;
    stationBoxCounter = 0;
    stationBoxCounted = false;
    stationLostCount = 0;
    params->resultsSnapshot(&lastResultsRevision);
}
