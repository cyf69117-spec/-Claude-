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
 * @file obstacle.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 全局障碍物检测（锥桶/行人）
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/obstacle.hpp"
#include <algorithm>
#include "utils/tools.hpp"

FsmObstacle::FsmObstacle(std::shared_ptr<Params> par)
    : params(par)
{
}

FsmObstacle::~FsmObstacle()
{
}

void FsmObstacle::run(Mat &img, bool allowReplan)
{
    std::uint64_t revision = 0;
    const std::vector<PredictResult> results = params->resultsSnapshot(&revision); // 结果与版本一起锁内拷贝
    const bool freshResults = revision != lastResultsRevision;
    lastResultsRevision = revision;
    params->ctrl.obstacleSlow = false;
    params->ctrl.obstacleAlarm = false;
    params->ctrl.obstacleStop = stopHeld;
    resultObs = PredictResult();

    // 先检查检测结果再检查边线：报警不能被缺边线、框大小或特殊路线屏蔽。
    // 只裁剪本地副本，保留AI发布的原始结果，防止贴边大框被尺寸条件丢弃。
    vector<PredictResult> visible;
    for (auto result : results)
    {
        if (result.type != LABEL_CONE && result.type != LABEL_PERSON)
            continue;
        params->ctrl.obstacleAlarm = true; // AI已确认类别就报警，不等待二次确认或改线
        if (result.width <= 0 || result.height <= 0)
            continue;
        const auto x1 = std::clamp(static_cast<long long>(result.x), 0LL, static_cast<long long>(COLSIMAGE));
        const auto x2 = std::clamp(static_cast<long long>(result.x) + result.width, 0LL, static_cast<long long>(COLSIMAGE));
        const auto y1 = std::clamp(static_cast<long long>(result.y), 0LL, static_cast<long long>(ROWSIMAGE));
        const auto y2 = std::clamp(static_cast<long long>(result.y) + result.height, 0LL, static_cast<long long>(ROWSIMAGE));
        if (x2 <= x1 || y2 <= y1)
            continue;
        result.x = static_cast<int>(x1);
        result.y = static_cast<int>(y1);
        result.width = static_cast<int>(x2 - x1);
        result.height = static_cast<int>(y2 - y1);
        visible.push_back(result);
    }
    if (visible.empty())
    {
        confirmCount = 0;
        if (params->ctrl.obstacleAlarm)
        {
            stopHeld = true; // 有目标类别但框无效，不能当作道路已经清空。
            clearResultsCount = 0;
        }
        else if (freshResults && ++clearResultsCount >= 3)
            stopHeld = false;
        params->ctrl.obstacleStop = stopHeld;
        return;
    }
    const auto holdStop = [this]()
    {
        stopHeld = true;
        clearResultsCount = 0;
        params->ctrl.obstacleStop = true;
        params->ctrl.obstacleSlow = true;
    };

    // 原120点门槛会在弯道或修线后直接漏掉障碍物；改为检查是否有可用边线。
    // 缺边线时不能判定绕行空间，输出零速度，不继续沿旧中心前进。
    if (params->track->pointsEdgeLeft.size() < 4 ||
        params->track->pointsEdgeRight.size() < 4)
    {
        confirmCount = 0;
        holdStop();
        return;
    }

    // 按目标所在图像行寻找边线，筛掉路边目标后再选最大的车道内障碍物。
    // 避免一个更大的路外行人遮蔽真正需要避让的锥桶。
    const auto obstacleRow = [this](int y)
    {
        size_t row = 0;
        int distance = ROWSIMAGE;
        for (size_t i = 0; i < params->track->pointsEdgeLeft.size(); i++)
        {
            const int d = std::abs(y - params->track->pointsEdgeLeft[i].x);
            if (d < distance)
            {
                distance = d;
                row = i;
            }
        }
        return std::min(row, params->track->pointsEdgeRight.size() - 1);
    };

    // 对进入本车道的目标生成绕行候选；路边目标仍报警，但不任意改线。
    vector<PredictResult> resultsObs;
    for (const auto &result : visible)
    {
        const size_t row = obstacleRow(result.y + result.height - 1); // 使用接地点附近的边线
        if (params->track->pointsEdgeRight[row].y <= params->track->pointsEdgeLeft[row].y)
        {
            holdStop(); // 边线交叉时不能把负间隙当成可用空间。
            return;
        }
        if (result.x + result.width > params->track->pointsEdgeLeft[row].y &&
            result.x < params->track->pointsEdgeRight[row].y)
            resultsObs.push_back(result);
    }
    if (resultsObs.size() <= 0)
    {
        confirmCount = 0;
        if (freshResults && ++clearResultsCount >= 3)
            stopHeld = false;
        params->ctrl.obstacleStop = stopHeld;
        return;
    }

    clearResultsCount = 0;

    // 选取距离最近的障碍物
    int areaMax = 0;
    int index = 0;
    for (int i = 0; i < resultsObs.size(); i++)
    {
        int area = resultsObs[i].width * resultsObs[i].height;
        if (area >= areaMax)
        {
            index = i;
            areaMax = area;
        }
    }
    const PredictResult &candidate = resultsObs[index];
    if (freshResults)
    {
        // 两批新推理连续确认同类且位置相近的目标后，才允许修改真实边线。
        const bool sameTarget = confirmCount > 0 && candidate.type == candidateLast.type &&
                                std::abs(candidate.x + candidate.width / 2 - candidateLast.x - candidateLast.width / 2) <= 48 &&
                                std::abs(candidate.y + candidate.height - candidateLast.y - candidateLast.height) <= 48;
        confirmCount = sameTarget ? std::min(confirmCount + 1, 2) : 1;
        candidateLast = candidate;
    }
    if (confirmCount < 2 || !allowReplan)
    {
        holdStop(); // 等待确认、倒车或专用路线不能改线时，也不能继续带速接近。
        return;
    }
    resultObs = candidate;

    // 使用与目标筛选一致的行，决定从障碍物空隙较大的一侧绕行。
    const size_t row = obstacleRow(resultObs.y + resultObs.height - 1);

    // 路径重规划
    int disLeft = resultsObs[index].x - params->track->pointsEdgeLeft[row].y;
    int disRight = params->track->pointsEdgeRight[row].y - (resultsObs[index].x + resultsObs[index].width);
    // 沿用原锥桶的一框宽余量；行人也必须让整个人体框处于绕行边界外。
    // 这是原图像规划余量，不代表实际车宽或制动距离，不能据此宣称实车安全距离。
    const int middle = (params->track->pointsEdgeLeft[row].y + params->track->pointsEdgeRight[row].y) / 2;
    const int limitRight = std::max(middle, resultObs.x + resultObs.width * 2);
    const int limitLeft = std::min(middle, resultObs.x - resultObs.width);
    const bool goRight = disRight >= disLeft;
    const int limit = goRight ? limitRight : limitLeft;
    if ((goRight && limit >= params->track->pointsEdgeRight[row].y) ||
        (!goRight && limit <= params->track->pointsEdgeLeft[row].y))
    {
        holdStop(); // 大目标占满可用侧向空间时停车，不能把贝塞尔曲线钳到另一条边线上。
        return;
    }
    for (const auto &other : resultsObs)
    {
        if ((goRight && other.x + other.width > limit) ||
            (!goRight && other.x < limit))
        {
            holdStop(); // 选定通道内还有其他目标时，不向另一个障碍物绕行。
            return;
        }
    }
    stopHeld = false;
    params->ctrl.obstacleStop = false;
    if (resultsObs[index].x + resultsObs[index].width > params->track->pointsEdgeLeft[row].y &&
        params->track->pointsEdgeRight[row].y > resultsObs[index].x &&
        goRight) //[1] 障碍物靠左
    {
        if (resultsObs[index].type == LABEL_PERSON) // 行人避障
        {
            curtailTracking(false);
            for (auto &point : params->track->pointsEdgeLeft)
                point.y = std::max(point.y, limitRight);
        }
        else
        {
            vector<PointX> points(4); // 三阶贝塞尔曲线
            points[0] = params->track->pointsEdgeLeft[row / 2];
            points[1] = {resultsObs[index].y + resultsObs[index].height, std::clamp(resultsObs[index].x + resultsObs[index].width * 2, 0, COLSIMAGE - 1)};
            points[2] = {(resultsObs[index].y + resultsObs[index].height + resultsObs[index].y) / 2, std::clamp(resultsObs[index].x + resultsObs[index].width * 2, 0, COLSIMAGE - 1)};
            if (resultsObs[index].y > params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() - 1].x)
                points[3] = params->track->pointsEdgeLeft[params->track->pointsEdgeLeft.size() - 1];
            else
                points[3] = {resultsObs[index].y, std::min(resultsObs[index].x + resultsObs[index].width, COLSIMAGE - 1)};

            params->track->pointsEdgeLeft.resize((size_t)row / 2); // 删除错误路线
            vector<PointX> repair = Bezier(0.01, points);          // 重新规划车道线
            for (int i = 0; i < repair.size(); i++)
                params->track->pointsEdgeLeft.push_back(repair[i]);
        }
        params->ctrl.obstacleSlow = true; // 避障独立减速，目标消失或功能停用即解除
    }
    else if (resultsObs[index].x + resultsObs[index].width > params->track->pointsEdgeLeft[row].y &&
             params->track->pointsEdgeRight[row].y > resultsObs[index].x &&
             !goRight) //[2] 障碍物靠右
    {
        if (resultsObs[index].type == LABEL_PERSON) // 行人避障
        {
            curtailTracking(true);
            for (auto &point : params->track->pointsEdgeRight)
                point.y = std::min(point.y, limitLeft);
        }
        else
        {
            vector<PointX> points(4); // 三阶贝塞尔曲线
            points[0] = params->track->pointsEdgeRight[row / 2];
            points[1] = {resultsObs[index].y + resultsObs[index].height, std::clamp(resultsObs[index].x - resultsObs[index].width * 2, 0, COLSIMAGE - 1)};
            points[2] = {(resultsObs[index].y + resultsObs[index].height + resultsObs[index].y) / 2, std::clamp(resultsObs[index].x - resultsObs[index].width * 2, 0, COLSIMAGE - 1)};
            if (resultsObs[index].y > params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() - 1].x)
                points[3] = params->track->pointsEdgeRight[params->track->pointsEdgeRight.size() - 1];
            else
                points[3] = {resultsObs[index].y, resultsObs[index].x};

            params->track->pointsEdgeRight.resize((size_t)row / 2); // 删除错误路线
            vector<PointX> repair = Bezier(0.01, points);           // 重新规划车道线
            for (int i = 0; i < repair.size(); i++)
                params->track->pointsEdgeRight.push_back(repair[i]);
        }
        params->ctrl.obstacleSlow = true; // 避障独立减速，目标消失或功能停用即解除
    }

    const auto &left = params->track->pointsEdgeLeft;
    const auto &right = params->track->pointsEdgeRight;
    for (const auto &point : left)
    {
        size_t nearest = 0;
        int distance = ROWSIMAGE;
        for (size_t i = 0; i < right.size(); ++i)
        {
            const int d = std::abs(right[i].x - point.x);
            if (d < distance) { distance = d; nearest = i; }
        }
        if (right.empty() || point.y >= right[nearest].y)
        {
            holdStop();
            return;
        }
    }

    if (!params->ctrl.obstacleSlow)
        return; // 未实际绕行时不裁剪原始边线。

    // 车道线切除顶部30%，避免弯道权重过大
    params->track->pointsEdgeLeft.resize(params->track->pointsEdgeLeft.size() * 0.7);
    params->track->pointsEdgeRight.resize(params->track->pointsEdgeRight.size() * 0.7);
}

void FsmObstacle::resetLap()
{
    resultObs = PredictResult();
    candidateLast = PredictResult();
    confirmCount = 0;
    // 记录当前发布版本，换圈前的旧结果不能作为新确认。
    params->resultsSnapshot(&lastResultsRevision);
    params->ctrl.obstacleSlow = false;
    params->ctrl.obstacleAlarm = false;
    params->ctrl.obstacleStop = stopHeld; // 换圈不能解除尚未确认消失的障碍物停车
    clearResultsCount = 0;
}

void FsmObstacle::show(Mat &img)
{
    if (resultObs.x > 0 && resultObs.y > 0)
    {
        cv::Rect rect(resultObs.x, resultObs.y, resultObs.width, resultObs.height);
        cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 1);
    }
}

void FsmObstacle::curtailTracking(bool left)
{
    if (left) // 向左侧缩进
    {
        if (params->track->pointsEdgeRight.size() > params->track->pointsEdgeLeft.size())
            params->track->pointsEdgeRight.resize(params->track->pointsEdgeLeft.size());

        for (int i = 0; i < params->track->pointsEdgeRight.size(); i++)
        {
            params->track->pointsEdgeRight[i].y = (params->track->pointsEdgeRight[i].y + params->track->pointsEdgeLeft[i].y) / 2;
        }
    }
    else // 向右侧缩进
    {
        if (params->track->pointsEdgeRight.size() < params->track->pointsEdgeLeft.size())
            params->track->pointsEdgeLeft.resize(params->track->pointsEdgeRight.size());

        for (int i = 0; i < params->track->pointsEdgeLeft.size(); i++)
        {
            params->track->pointsEdgeLeft[i].y = (params->track->pointsEdgeRight[i].y + params->track->pointsEdgeLeft[i].y) / 2;
        }
    }
}
