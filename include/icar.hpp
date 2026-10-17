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
 * @file icar.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 智能汽车控制（TOP）
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/busy.hpp"
#include "fsm/fork.hpp"
#include "fsm/park.hpp"
#include "fsm/stop.hpp"
#include "fsm/cross.hpp"
#include "fsm/yfork.hpp"
#include "fsm/station.hpp"
#include "fsm/slow.hpp"
#include "fsm/obstacle.hpp"
#include "com/client.hpp"
#include "utils/detection.hpp"
#include "utils/show.hpp"
#include "utils/loop.hpp"
#include "ctrl/predeal.hpp"
#include "ctrl/center.hpp"
#include "ctrl/motion.hpp"
#include <stdexcept>
#include <atomic>

using namespace std;
using namespace cv;

class Icar
{
private:
    /**
     * @brief 状态机管理
     *
     */
    struct FsmFactory
    {
        shared_ptr<FsmBusy> busy;               // 避障控制
        shared_ptr<FsmPark> park;               // 停车场控制
        shared_ptr<FsmStop> stop;               // 停车区控制
        shared_ptr<FsmCross> cross;             // 斑马线停车控制
        shared_ptr<FsmFork> fork;               // 停车场岔路控制
        shared_ptr<FsmSlow> slow;               // 慢行区控制
        shared_ptr<FsmObstacle> obstacle;       // 全局障碍物检测（锥桶/行人）
        shared_ptr<FsmYfork> yfork;             // Y型岔路口控制
        shared_ptr<FsmStation> station;         // 停靠站控制
    };

    FsmFactory fsmFactory;                // 状态机管理
    shared_ptr<Predeal> predeal;          // 图像预处理类
    shared_ptr<Show> show;                // 初始化UI显示窗口
    shared_ptr<cv::VideoCapture> capture; // Opencv相机类
    shared_ptr<Detection> detection;      // 目标检测类
    shared_ptr<Client> client;            // TCP客户端通信类
    shared_ptr<Params> params;            // 车辆状态参数（FSM共享传递）
    shared_ptr<Loops> loops;              // 子线程循环
    shared_ptr<Center> center;            // 控制中心处理类
    shared_ptr<Motion> motion;            // 运动控制器

    int lastLap = 0; // 上一圈号（检测圈变更时复位FSM）
    std::atomic<bool> stopping{false};
    std::atomic<bool> inferenceFailed{false};
    bool poseContextValid = false;
    bool poseBackLast = false, poseParkingLast = false, poseStopLast = false;

    void stopVehicle() noexcept
    {
        // 最终停车命令不再经过FSM或姿态计算，防止停车被后续逻辑覆盖。
        if (params)
        {
            params->shutdownRequested = true;
            params->ctrl.stop = true;
            params->ctrl.speed = 0.0f;
            params->ctrl.servo = PWMSERVOMID;
        }
        if (motion)
            motion->resetPose();
        try
        {
            if (client)
                client->carControl(0.0f, PWMSERVOMID);
        }
        catch (...) {} // 析构或异常处理中仍须继续回收线程，不能再次抛出异常。
    }

    void updatePoseContext()
    {
        // 倒车、停车恢复和停车场退出时，旧P/D历史不得带入新模式。
        if (!poseContextValid ||
            params->ctrl.back != poseBackLast || params->ctrl.stop != poseStopLast ||
            (poseParkingLast && !params->ctrl.parking))
            motion->resetPose();
        poseContextValid = true;
        poseBackLast = params->ctrl.back;
        poseParkingLast = params->ctrl.parking;
        poseStopLast = params->ctrl.stop;
    }

    // 全局共享数据链
    cv::Mat imgShare;
    std::mutex mtxImg;
    std::condition_variable cvImg;
    std::atomic<bool> readyImg{false};

    /**
     * @brief 鼠标的事件回调函数
     *
     */
    static void callbackMouse(int event, int x, int y, int flags, void *userdata)
    {
        Icar *self = static_cast<Icar *>(userdata);
        if (self)
            self->handleMouse(event, x, y, flags);
    }
    void handleMouse(int event, int x, int y, int flags)
    {
        double value;
        switch (event)
        {
        case cv::EVENT_MOUSEWHEEL: // 鼠标滑球
        {
            value = cv::getMouseWheelDelta(flags); // 获取滑球滚动值
            if (value > 0)
                show->index++;
            else if (value < 0)
                show->index--;

            if (show->index < 0)
                show->index = 0;
            if (show->index > show->frameMax)
                if (show->index > show->frameMax)
                    show->index = show->frameMax;
            break;
        }
        default:
            break;
        }
    }

    /**
     * @brief AI 模型推理
     *
     */
    void runModel()
    {
        std::unique_lock<std::mutex> lock(mtxImg);
        cvImg.wait(lock, [this] { return stopping.load() || readyImg.load(); });
        if (stopping.load())
            return;
        cv::Mat img = imgShare.clone();
        readyImg = false;
        lock.unlock();
        try
        {
            detection->inference(img);
            params->updateResults(detection->results);
        }
        catch (...)
        {
            params->updateResults({});
            inferenceFailed.store(true); // 主线程统一停车，AI线程不写ctrl或发送命令。
        }
    }

    /**
     * @brief 有限状态机任务执行
     *
     */
    void runFsm(Mat &img)
    {
        params->ctrl.obstacleSlow = false; // 即使避障本帧未调用，也不能保留上一帧减速
        bool warningDue = false; // 同帧多来源报警合并，避免重复蜂鸣指令
        if (params->shutdownRequested || params->safetyStop)
            return;
        // 圈数变更时复位所有FSM状态
        if (lastLap != params->currentLap)
        {
            lastLap = params->currentLap;
            motion->resetPose();
            params->ctrl.slow = false;
            fsmFactory.stop->resetLap();
            fsmFactory.park->resetLap();
            fsmFactory.fork->resetLap();
            fsmFactory.yfork->resetLap();
            fsmFactory.slow->resetLap();
            fsmFactory.busy->resetLap();
            fsmFactory.station->resetLap();
            fsmFactory.obstacle->resetLap();
            params->alertCountdown = 0;                // 复位蜂鸣器报警
            params->alertDecelCount = 0;               // 复位报警减速
            params->busyAlertCountdown = 0;            // 复位施工区蜂鸣
        }

        if (params->mode == FsmMode::STATION ||
            params->mode == FsmMode::FORK ||
            params->mode == FsmMode::BUSY ||
            params->mode == FsmMode::SLOW ||
            params->mode == FsmMode::YFORK) // 状态复位
            params->mode = FsmMode::NORMAL;

        if (!params->config.currentLapConfig)
        {
            params->shutdownRequested = true;
            return;
        }
        // 根据当前圈配置设置功能使能（覆盖全局配置）
        params->config.fork = params->config.currentLapConfig->fork;
        params->config.park = params->config.currentLapConfig->park;
        params->config.busy = params->config.currentLapConfig->busy;
        params->config.slow = params->config.currentLapConfig->slow;
        params->config.stop = params->config.currentLapConfig->stop;
        params->config.cross = params->config.currentLapConfig->cross;
        params->config.yfork = params->config.currentLapConfig->yfork;
        params->config.station = params->config.currentLapConfig->station; // 施工区停靠由语言任务决定
        params->config.obstacle = params->config.currentLapConfig->obstacle;
        if (!params->config.slow)
            params->ctrl.slow = false;

        fsmFactory.stop->run(img); // 停车区识别与规划
        params->mode = fsmFactory.stop->getMode();
        fsmFactory.cross->run(img); // 斑马线停车识别与规划
        params->mode = fsmFactory.cross->getMode();
        if (params->shutdownRequested)
            return; // 终点停车请求不能再进入其他FSM

        // ===== 路线隔离：仅在当前圈使能时调用对应FSM =====
        if (params->config.park)
        {
            if (params->mode == FsmMode::NORMAL || params->mode == FsmMode::PARK || params->mode == FsmMode::CROSS)
            {
                fsmFactory.park->run(img);
                FsmMode mode = fsmFactory.park->getMode();
                if (mode != FsmMode::NORMAL)
                    params->mode = mode;
            }
        }
        if (params->config.fork)
        {
            if (params->mode == FsmMode::NORMAL)
            {
                fsmFactory.fork->run(img);
                params->mode = fsmFactory.fork->getMode();
            }
        }
        if (params->config.yfork)
        {
            if (params->mode == FsmMode::NORMAL)
            {
                fsmFactory.yfork->run(img);
                params->mode = fsmFactory.yfork->getMode();
            }
        }
        if (params->config.slow)
        {
            fsmFactory.slow->run(img);
            if (params->mode == FsmMode::NORMAL)
                params->mode = fsmFactory.slow->getMode();
        }
        if (params->config.busy)
        {
            fsmFactory.busy->run(img);
            if (params->mode == FsmMode::NORMAL)
                params->mode = fsmFactory.busy->getMode();
            // 施工区鸣笛（每秒3次，间隔10帧≈333ms）
            if (params->busyAlertCountdown > 0)
            {
                if (params->busyAlertCountdown % 10 == 0)
                    warningDue = true;
                params->busyAlertCountdown--;
            }
        }
        if (params->config.station)
        {
            fsmFactory.station->run(img);
            FsmMode stationMode = fsmFactory.station->getMode();
            // 不覆盖YFORK模式，让yfork状态能正常显示
            if (stationMode != FsmMode::NORMAL && params->mode != FsmMode::YFORK)
                params->mode = stationMode;
        }

        // 所有路线都检查障碍物，专用轨迹无法绕行时报警并输出零速度。
        const bool allowReplan = params->config.obstacle && !params->ctrl.back &&
                                 !params->ctrl.fitting && params->mode != FsmMode::PARK &&
                                 params->mode != FsmMode::YFORK;
        fsmFactory.obstacle->run(img, allowReplan);
        if (params->ctrl.obstacleStop)
        {
            params->ctrl.speed = 0.0f;
            params->ctrl.servo = PWMSERVOMID;
            if (!client->carControl(0.0f, PWMSERVOMID))
            {
                stopVehicle();
                return;
            }
        } // 先请求零速度，再发送报警音效，且不等待中心线拟合或绘图。
        if (params->ctrl.obstacleAlarm && params->alertCountdown <= 0)
            params->alertCountdown = 30;

        // 其他报警目标保持原配置，避障报警与施工区报警统一在本帧发送一次。
        if (params->config.alertTarget != "none")
        {
            const std::vector<PredictResult> results = params->resultsSnapshot(); // 报警块快照，锁内拷贝后独立遍历
            int alertLabel = -1;
            if (params->config.alertTarget == "cone")
                alertLabel = LABEL_CONE;
            else if (params->config.alertTarget == "person")
                alertLabel = LABEL_PERSON;
            else if (params->config.alertTarget == "busy")
                alertLabel = LABEL_BUSY;
            else if (params->config.alertTarget == "limit")
                alertLabel = LABEL_LIMIT;
            else if (params->config.alertTarget == "unlimit")
                alertLabel = LABEL_UNLIMIT;
            else if (params->config.alertTarget == "park")
                alertLabel = LABEL_PARK;

            // 行人和锥桶由独立报警标志处理，不受单个alertTarget配置限制。
            if (alertLabel == LABEL_CONE || alertLabel == LABEL_PERSON)
                alertLabel = -1;

            // 施工区由FSM自身负责鸣笛，不重复触发
            if (alertLabel == LABEL_BUSY && params->config.currentLapConfig &&
                params->config.currentLapConfig->busy)
                alertLabel = -1;

            if (alertLabel >= 0)
            {
                bool targetFound = false;
                {
                    for (auto &r : results)
                    {
                        bool posOk = (alertLabel == LABEL_LIMIT || alertLabel == LABEL_PARK)
                                         ? (r.x > COLSIMAGE * 0.8 || r.x + r.width < COLSIMAGE * 0.2) // 左右两侧1/5
                                         : (r.y + r.height) > ROWSIMAGE * 0.2;                        // 其他：底部4/5区域
                        if (r.type == alertLabel && posOk)
                        {
                            targetFound = true;
                            break;
                        }
                    }
                }
                if (targetFound && params->alertCountdown <= 0)
                    params->alertCountdown = 30; // 启动最少1秒倒计时
            }
        }
        // 即使alertTarget为none，识别到行人/锥桶也发送现有报警音效。
        if (params->alertCountdown > 0)
        {
            if (params->alertCountdown % 10 == 0) // 每~333ms鸣笛一次，最少3次
                warningDue = true;
            params->alertCountdown--;
        }
        if (warningDue)
            client->buzzerSound(client->BUZZER_WARNNING);

        {
            const std::vector<PredictResult> results = params->resultsSnapshot(); // 减速块独立取快照，允许看到更新结果
            bool decelFound = false;
            {
                for (auto &r : results)
                {
                    bool posOk = false;
                    if (r.type == LABEL_LIMIT || r.type == LABEL_PARK)
                        posOk = (r.x > COLSIMAGE * 0.8 || r.x + r.width < COLSIMAGE * 0.2); // 左右两侧1/5
                    else if (r.type == LABEL_CONE || r.type == LABEL_PERSON || r.type == LABEL_UNLIMIT)
                        posOk = (r.y + r.height) > ROWSIMAGE * 0.2; // 底部4/5区域
                    else if (r.type == LABEL_BUSY)
                    {
                        if (params->config.currentLapConfig && params->config.currentLapConfig->busy)
                            continue;
                        posOk = (r.y + r.height) > ROWSIMAGE * 0.2;
                    }
                    else
                        continue;

                    if (posOk)
                    {
                        decelFound = true;
                        break;
                    }
                }
            }
            if (decelFound)
                params->alertDecelCount = 5;
            else if (params->alertDecelCount > 0)
                params->alertDecelCount--;
        }

        if (params->mode != params->modeLast && params->alertCountdown <= 0 && !warningDue && !params->ctrl.obstacleAlarm)
        {
            client->buzzerSound(client->BUZZER_DING); // 提示音效
            params->modeLast = params->mode;
        }
    }

public:
    /**
     * @brief 参数初始化
     *
     */
    Icar()
    {
        params = make_shared<Params>();                        // 初始化参数
        center = make_shared<Center>();                        // 控制中心处理类
        motion = make_shared<Motion>();                        // 运动控制器
        predeal = make_shared<Predeal>(params->config.binary); // 图像预处理类
        detection = make_shared<Detection>(params->config.model,
                                           params->config.score); // AI模型初始化

        // 初始化TCP通信客户端
        client = make_shared<Client>();
        if (!client->start())
        {
            printf("[Error]: Socket init failed!!!\n");
            throw std::runtime_error("Icar initialization failed");
        }
        client->buzzerSound(client->BUZZER_OK); // 提示音效

        // 相机初始化
        // USB摄像头初始化
        if (params->config.debug)
            capture = make_shared<cv::VideoCapture>(params->config.video); // 打开本地视频
        else
            capture = make_shared<cv::VideoCapture>("/dev/video0"); // 打开摄像头
        if (!capture->isOpened())
        {
            printf("[Error]: Can not open video device!!!\n");
            throw std::runtime_error("Icar initialization failed");
        }
        capture->set(cv::CAP_PROP_FRAME_WIDTH, COLSCAMERA);  // 设置图像分辨率
        capture->set(cv::CAP_PROP_FRAME_HEIGHT, ROWSCAMERA); // 设置图像分辨率
        capture->set(cv::CAP_PROP_FPS, 30);                  // 设置帧率

        if (params->config.debug)
        {
            show = make_shared<Show>(4); // 调试UI初始化
            show->frameMax = capture->get(cv::CAP_PROP_FRAME_COUNT) - 1;
            cv::createTrackbar("Frame", "ICAR", &show->index, show->frameMax, [](int, void *) {}); // 创建Opencv图像滑条控件
            cv::setMouseCallback("ICAR", this->callbackMouse);                                     // 创建鼠标键盘快捷键事件
        }

        // FSM有限状态机初始化
        fsmFactory.busy = make_shared<FsmBusy>(params);         // 避障控制实例化
        fsmFactory.park = make_shared<FsmPark>(params);         // 停车场控制实例化
        fsmFactory.stop = make_shared<FsmStop>(params);         // 斑马线停车控制实例化
        fsmFactory.cross = make_shared<FsmCross>(params);       // 斑马线停车控制实例化
        fsmFactory.fork = make_shared<FsmFork>(params);         // 停车场岔路控制实例化
        fsmFactory.slow = make_shared<FsmSlow>(params);         // 慢行区控制实例化
        fsmFactory.obstacle = make_shared<FsmObstacle>(params); // 全局障碍物检测实例化
        fsmFactory.yfork = make_shared<FsmYfork>(params);       // Y型岔路口控制实例化
        fsmFactory.station = make_shared<FsmStation>(params);   // 停靠站控制实例化

        // 启动AI推理子线程
        loops = make_shared<Loops>("LoopAI", 1.f / 30.f, std::bind(&Icar::runModel, this));
        loops->start(); // RL开始推理

        printf("[OK]: Params initial succeed!!!\n");
    };
    ~Icar()
    {
        stopVehicle(); // 先停车，再等待可能仍在推理的线程结束
        {
            std::lock_guard<std::mutex> lock(mtxImg);
            stopping.store(true);
        }
        cvImg.notify_all(); // 唤醒runModel自己的等待条件，而不只是Loops的周期等待
        if (loops)
            loops->shutdown();
        if (client)
            client->closeClient();
    };

    bool isRunning() const { return !params->shutdownRequested; }

    /**
     * @brief 程序主循环
     *
     */
    void running()
    {
        if (params->shutdownRequested || inferenceFailed.load() || !client->isConnected())
        {
            stopVehicle();
            return;
        }
        //[01] 视频源读取
        cv::Mat img;
        if (params->config.debug) // 综合显示调试UI窗口
        {
            if (show->indexLast == show->index) // 图像帧未更新
            {
                if (client->keypress)
                {
                    client->buzzerSound(client->BUZZER_FINISH); // 祖传提示音效
                    printf("-----> System Exit!!! <-----\n");
                    stopVehicle();
                    return; // 交还主循环正常退出
                }
                show->show();        // 显示综合绘图
                client->sendHeart(); // 发送给服务器在线心跳
                usleep(10 * 1000);   // us延迟
                return;
            }

            capture->set(cv::CAP_PROP_POS_FRAMES, show->index); // 设置读取帧
            if (!capture->read(img))
            {
                stopVehicle();
                return;
            }
            show->indexLast = show->index;
        }
        else if (!capture->read(img))
        {
            stopVehicle();
            return;
        }

        //[02] 图像存储
        if (params->config.saveImg && !params->config.debug) // 存储原始图像
            savePicture(img);
        else if (params->config.saveImg && params->config.debug) // 存储调式图像
            show->save = true;

        //[03] 图像预处理
        cv::Mat imgBin;
        predeal->correction(img); // 图像矫正
        /*---------------子线程共享数据，避免浅拷贝-----------------*/
        {
            std::lock_guard<std::mutex> lock(mtxImg);
            imgShare = img.clone();
            readyImg = true;
        } // 只保护图像发布，不能持锁覆盖余下整帧
        cvImg.notify_one();
        /*-------------------------------------------------------*/
        imgBin = predeal->binaryzation(img); // 图像二值化

        //[04] 赛道识别
        params->track->handle(imgBin);
        if (params->config.debug)
        {
            show->setNewWindow(1, "Bin", imgBin);
            cv::Mat imgTrack = img.clone();
            params->track->drawImage(imgTrack); // 图像绘制赛道识别结果
            show->setNewWindow(2, "Track", imgTrack);
            if (params->config.saveIpm && params->config.saveImg)
            {
                cv::Mat imgIpm;
                ipm.homography(imgTrack, imgIpm);
                savePicture(imgIpm); // 保存图像
            }
        }

        //[05] 有限状态机任务执行
        params->ctrl.fitting = false;
        runFsm(imgBin);
        if (params->shutdownRequested || params->safetyStop || inferenceFailed.load())
        {
            stopVehicle();
            return;
        }

        if (params->ctrl.obstacleStop)
        {
            motion->resetPose();
            params->ctrl.speed = 0.0f;
            params->ctrl.servo = PWMSERVOMID;
            client->carControl(0.0f, PWMSERVOMID); // 请求零速度；不虚构下位机主动刹车接口
            return;
        }

        //[06] 控制中心计算
        center->fitting(params);

        //[07] 车辆运动控制
        motion->outlineCheck(params); // 唯一出界检测路径，每帧更新一次冷却和丢线计数
        if (params->shutdownRequested || params->safetyStop)
        {
            stopVehicle();
            return;
        }
        updatePoseContext();
        motion->poseControl(params);
        motion->speedControl(params);

        //[08] 综合显示调试UI窗口
        if (params->config.debug)
        {
            detection->drawBox(img, params->resultsSnapshot()); // 使用快照绘制，避免读取AI线程正在修改的成员
            center->drawImage(params, img); // 图像绘制控制路径
            motion->drawImage(params, img); // 图像绘制速度
            show->setNewWindow(3, "Ctrl", img);

            // 特殊区域图像处理结果显示
            Mat imgRes = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
            fsmFactory.busy->show(imgRes);
            fsmFactory.park->show(imgRes);
            fsmFactory.stop->show(imgRes);
            fsmFactory.cross->show(imgRes);
            fsmFactory.fork->show(imgRes);
            fsmFactory.slow->show(imgRes);
            fsmFactory.obstacle->show(imgRes);
            fsmFactory.yfork->show(imgRes);
            show->setNewWindow(4, "FSM", imgRes);
        }
        else // 实车控制
        {
            // 每帧发送自动控制指令（MCU需要持续PWM更新）
            if (!client->carControl(params->ctrl.speed, params->ctrl.servo))
                stopVehicle(); // 发送失败会断开连接，由Boot侧立即补发停车帧
        }
    }
};
