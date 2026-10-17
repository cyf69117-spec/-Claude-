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
 * @file busy.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 避障（施工区）控制
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/busy.hpp"

/**
 * @brief Construct a new Fsm Park
 *
 * @param par
 */
FsmBusy::FsmBusy(std::shared_ptr<Params> par)
    : FSMState(FsmMode::BUSY, par)
{
}

/**
 * @brief Destroy the Fsm Park
 *
 */
FsmBusy::~FsmBusy()
{
}

/**
 * @brief 检查状态切换
 *
 * @return FsmMode 切换后的状态
 */
FsmMode FsmBusy::getMode()
{
    if (step != Step::NONE && step != Step::RETURN && params->config.busy &&
        params->config.currentLapConfig && params->config.currentLapConfig->busy)
        return FsmMode::BUSY;
    return FsmMode::NORMAL;
}

// 施工区为主赛道左侧第一条支路：左转进区，绕长岛循迹，再左转回主赛道。
void FsmBusy::run(Mat &img)
{
    std::uint64_t revision = 0;
    const auto results = params->resultsSnapshot(&revision); // 每帧只取一次带锁快照
    const bool fresh = revision != lastResultsRevision;
    lastResultsRevision = revision;
    if (!params->config.busy || !params->config.currentLapConfig ||
        !params->config.currentLapConfig->busy)
    {
        resetLap();
        return;
    }
    if (step == Step::RETURN || params->ctrl.stop || params->ctrl.back || params->ctrl.obstacleStop)
        return; // 已完成本支路不再重复进区；停车期间不能让转向计时继续推进。

    bool busyVisible = false, entryNear = false, leftVisible = false;
    for (const auto &r : results)
    {
        if (r.type == LABEL_BUSY && r.width > 0 && r.height > 0)
        {
            busyVisible = true;
            // 沿用原停车场岔路标志的近端判定，只用于触发入口引导，不换算米数。
            entryNear = entryNear || (r.y + r.height / 2) > ROWSIMAGE * 0.35;
        }
        if (r.type == LABEL_LEFT && r.width < 100 && r.height < 120 &&
            r.width > 0 && r.height > 0 && (r.y + r.height / 2) > ROWSIMAGE * 0.27)
            leftVisible = true;
    }

    switch (step)
    {
    case Step::NONE:
        if (!fresh)
            return;
        if (busyVisible)
        {
            ++countRec;
            countSes = 0;
        }
        else if (++countSes > 6)
            countRec = 0;
        if (countRec < 3)
            return;
        step = Step::APPROACH;
        params->busyZone = true;
        params->busyGuiding = true;
        params->busyRouteCompleted = false;
        params->stationStarted = false;
        params->stationStopCompleted = false;
        params->busyAlertCountdown = 40;
        turnFrames = signLost = laneFrames = 0;
        // 已确认且标志到近端时，本帧即可进入左转阶段。
        [[fallthrough]];
    case Step::APPROACH:
        if (++turnFrames > TURN_TIMEOUT_FRAMES)
        {
            routeFailed();
            return;
        }
        if (!entryNear)
            return;
        step = Step::ENTER;
        turnFrames = signLost = laneFrames = 0;
        [[fallthrough]];
    case Step::ENTER:
        ++turnFrames;
        if (fresh)
            signLost = busyVisible ? 0 : signLost + 1;
        laneFrames = laneVisible() ? laneFrames + 1 : 0; // 检查本帧真实边线，再决定是否补线
        if (turnFrames >= ENTRY_MIN_FRAMES && signLost >= 3 && laneFrames >= 3)
        {
            step = Step::TRACK;
            params->busyGuiding = false;
            turnFrames = 0;
            exitClear = exitRec = 0;
            exitArmed = false;
            return; // 恢复真实车道线，按施工区道路绕过长岛。
        }
        if (turnFrames > TURN_TIMEOUT_FRAMES)
        {
            routeFailed();
            return;
        }
        guideLeftTurn(true);
        return;
    case Step::TRACK:
    {
        // 入口箭头必须离开后才能确认出口箭头，不能刚左转进区就立即左转退出。
        if (fresh)
        {
            exitClear = leftVisible ? 0 : exitClear + 1;
            if (exitClear >= 3)
                exitArmed = true;
        }
        const auto &lap = *params->config.currentLapConfig;
        const bool stopRequested = lap.station && lap.busyStopEnable && lap.busyStopPoint > 0;
        if (stopRequested && !params->stationStopCompleted)
            return; // 按语言命令停在中间/出口后，才允许离开施工区。
        if (stopRequested && lap.busyStopPoint == 1 && stationExitCooldown < 30)
        {
            ++stationExitCooldown; // 保留原中间停靠结束后的1秒出口屏蔽。
            return;
        }
        if (!exitArmed || !fresh)
            return;
        exitRec = leftVisible ? exitRec + 1 : 0;
        if (exitRec < 2)
            return;
        step = Step::EXIT;
        params->busyGuiding = true;
        turnFrames = signLost = laneFrames = 0;
        [[fallthrough]];
    }
    case Step::EXIT:
        ++turnFrames;
        if (fresh)
            signLost = leftVisible ? 0 : signLost + 1;
        laneFrames = laneVisible() ? laneFrames + 1 : 0;
        if (turnFrames >= EXIT_MIN_FRAMES && signLost >= 3 && laneFrames >= 3)
        {
            step = Step::RETURN;
            params->busyZone = false;
            params->busyGuiding = false;
            params->busyRouteCompleted = true;
            params->ctrl.countAcc = 0;
            return; // 回到主赛道后循迹至斑马线，由cross统一计圈/终点停车。
        }
        if (turnFrames > TURN_TIMEOUT_FRAMES)
        {
            routeFailed();
            return;
        }
        guideLeftTurn(false);
        return;
    case Step::RETURN:
        return;
    }
}

bool FsmBusy::laneVisible() const
{
    const auto &left = params->track->pointsEdgeLeft;
    const auto &right = params->track->pointsEdgeRight;
    return left.size() >= ROWSIMAGE / 4 && right.size() >= ROWSIMAGE / 4 &&
           left.front().y < right.front().y && left.back().y < right.back().y;
}

void FsmBusy::guideLeftTurn(bool entering)
{
    // 复用本工程停车场入口和原施工区出口的左转补线，仍经center→PD输出舵机。
    // 只改车道线，不直接写舵机PWM，不假设照片中的像素对应实际转弯距离。
    const PointX startL(ROWSIMAGE - 10, 1), endL(ROWSIMAGE / 3, 1);
    const PointX midL((startL.x + endL.x) * 0.3, (startL.y + endL.y) / 2);
    const PointX startR(ROWSIMAGE - 10, int(COLSIMAGE * 0.8));
    const PointX endR(ROWSIMAGE / 3, entering ? 30 : 5);
    const PointX midR((startR.x + endR.x) * (entering ? 0.5 : 0.3),
                      (startR.y + endR.y) * (entering ? 0.35 : 0.5));
    params->track->pointsEdgeLeft = Bezier(0.008, {startL, midL, endL});
    params->track->pointsEdgeRight = Bezier(0.008, {startR, midR, endR});
}

void FsmBusy::routeFailed()
{
    params->ctrl.stop = true;
    params->ctrl.speed = 0.0f;
    params->safetyStop = true;
    params->shutdownRequested = true; // 超时通过现有安全退出路径发停车帧，不假装任务完成。
}

void FsmBusy::show(Mat &img)
{
    if (getMode() != FsmMode::BUSY)
        return;
    putText(img, "[1] Busy " + std::to_string(static_cast<int>(step)), Point(COLSIMAGE / 2 - 50, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);
}

void FsmBusy::resetLap()
{
    step = Step::NONE;
    countRec = countSes = turnFrames = laneFrames = signLost = 0;
    exitRec = exitClear = stationExitCooldown = 0;
    exitArmed = false;
    params->resultsSnapshot(&lastResultsRevision);
    params->busyZone = false;
    params->busyGuiding = false;
    params->busyRouteCompleted = false;
    params->busyAlertCountdown = 0;
}
