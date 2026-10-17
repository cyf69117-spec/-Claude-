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
 * @file busy.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 避障（施工区）控制
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/fsm.hpp"

/**
 * @brief 避障（施工区）控制
 *
 */
class FsmBusy : public FSMState
{
public:
    FsmBusy(std::shared_ptr<Params> par);
    ~FsmBusy();
    void run(Mat &img);
    void show(Mat &img);
    FsmMode getMode();
    void resetLap();

private:
    enum class Step { NONE, APPROACH, ENTER, TRACK, EXIT, RETURN };
    Step step = Step::NONE;
    int countRec = 0, countSes = 0;
    int turnFrames = 0, laneFrames = 0, signLost = 0;
    int exitRec = 0, exitClear = 0, stationExitCooldown = 0;
    bool exitArmed = false;
    std::uint64_t lastResultsRevision = 0;

    // 初始转向尺度沿用工程停车场入库24帧和原施工区出口15帧；需要实车标定。
    // 超时只停车，不把计时到期当作已经进入施工区或回到主赛道。
    static constexpr int ENTRY_MIN_FRAMES = 24;
    static constexpr int EXIT_MIN_FRAMES = 15;
    static constexpr int TURN_TIMEOUT_FRAMES = 90;
    void guideLeftTurn(bool entering);
    bool laneVisible() const;
    void routeFailed();
};
