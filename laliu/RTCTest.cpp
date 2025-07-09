// RTCTest.cpp
#include "RTCTest.h"
#include <libavutil/time.h>
#include <QTimer>          // ✅ 新增
#include <QElapsedTimer>   // ✅ 新增
#include <numeric>         // ✅ 新增（用于黑帧平均亮度计算）
#include <string> 
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

cv::Mat I420ToBGR(AVFrame* frame) {
    cv::Mat yuv(frame->height * 3 / 2, frame->width, CV_8UC1);
    memcpy(yuv.data, frame->data[0], frame->width * frame->height); // Y
    memcpy(yuv.data + frame->width * frame->height, frame->data[1], frame->width * frame->height / 4); // U
    memcpy(yuv.data + frame->width * frame->height + frame->width * frame->height / 4,
        frame->data[2], frame->width * frame->height / 4); // V

    cv::Mat bgr;
    cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_I420);
    return bgr;
}
void annotateRedBlue(cv::Mat& bgr) {
    // 转 HSV
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    // 红色范围（HSV，注意两个范围）
    cv::Mat mask_red1, mask_red2, mask_red;
    cv::inRange(hsv, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), mask_red1);
    cv::inRange(hsv, cv::Scalar(160, 70, 50), cv::Scalar(180, 255, 255), mask_red2);
    cv::bitwise_or(mask_red1, mask_red2, mask_red);

    // 蓝色范围
    cv::Mat mask_blue;
    cv::inRange(hsv, cv::Scalar(100, 70, 50), cv::Scalar(130, 255, 255), mask_blue);

    // 找轮廓并画框
    std::vector<std::vector<cv::Point>> contours;

    // 红色
    cv::findContours(mask_red, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (auto& c : contours) {
        if (cv::contourArea(c) > 500)  // 排除小干扰
            cv::rectangle(bgr, cv::boundingRect(c), cv::Scalar(0, 0, 255), 2);
    }

    // 蓝色
    cv::findContours(mask_blue, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (auto& c : contours) {
        if (cv::contourArea(c) > 500)
            cv::rectangle(bgr, cv::boundingRect(c), cv::Scalar(255, 0, 0), 2);
    }
}
void BGRToI420(const cv::Mat& bgr, AVFrame* frame) {
    cv::Mat yuv_i420;
    cv::cvtColor(bgr, yuv_i420, cv::COLOR_BGR2YUV_I420);

    int y_size = frame->width * frame->height;
    int uv_size = y_size / 4;
    memcpy(frame->data[0], yuv_i420.data, y_size);
    memcpy(frame->data[1], yuv_i420.data + y_size, uv_size);
    memcpy(frame->data[2], yuv_i420.data + y_size + uv_size, uv_size);
}

RTCTest::RTCTest(QWidget* parent) : QMainWindow(parent) {
    ui.setupUi(this);
    qRegisterMetaType<std::string>();
    qRegisterMetaType<bytertc::MediaStreamType>("bytertc::MediaStreamType");
    qRegisterMetaType<bytertc::StreamRemoveReason>("bytertc::StreamRemoveReason");

    this->resize(1980, 1080);
    QWidget* centralWidget = new QWidget(this);
    QHBoxLayout* lay = new QHBoxLayout(centralWidget);
    widget_local = new QWidget(this);
    widget_remote = new QWidget(this);
    lay->addWidget(widget_local);
    lay->addWidget(widget_remote);
    widget_local->setFixedSize(800, 600);
    widget_remote->setFixedSize(800, 600);
    widget_local->show();
    widget_remote->show();
    this->setCentralWidget(centralWidget);

    if (m_appid.empty() || m_uid.empty() || m_roomid.empty()) {
        QMessageBox::warning(this, "提示", "paras is empty", QMessageBox::Ok);
        return;
    }

    /* -------------------- 事件处理 -------------------- */
    m_handler = std::make_shared<EventHandler>();
    connect(m_handler.get(), &EventHandler::sigRoomStateChanged, this,
        [this](std::string roomid, std::string uid, int state, std::string extra_info) {
            qDebug() << Q_FUNC_INFO << "roomid=" << QString::fromStdString(roomid)
                << ",uid=" << QString::fromStdString(uid) << ",state=" << state;
        });

    connect(m_handler.get(), &EventHandler::sigUserPublishStream, this,
        [this](std::string uid, bytertc::MediaStreamType type) {
            qDebug() << Q_FUNC_INFO << "onUserPublishStream,uid="
                << QString::fromStdString(uid) << ",type=" << type;
            if (m_video) {
                bytertc::VideoCanvas cas;
                bytertc::RemoteStreamKey key{ m_roomid.c_str(), uid.c_str(),
                                              bytertc::kStreamIndexMain };
                cas.background_color = 0;
                cas.render_mode = bytertc::RenderMode::kRenderModeFit;
                cas.view = (void*)widget_remote->winId();
                m_video->setRemoteVideoCanvas(key, cas);
            }
        });

    connect(m_handler.get(), &EventHandler::sigUserUnpublishStream, this,
        [this](std::string uid, bytertc::MediaStreamType type,
            bytertc::StreamRemoveReason reason) {
                qDebug() << Q_FUNC_INFO << "onUserUnpublishStream,uid="
                    << QString::fromStdString(uid) << ",type=" << type
                    << ",reason=" << reason;
                if (m_video) {
                    bytertc::RemoteStreamKey key{ m_roomid.c_str(), uid.c_str(),
                                                  bytertc::kStreamIndexMain };
                    bytertc::VideoCanvas cas;
                    cas.view = nullptr;
                    m_video->setRemoteVideoCanvas(key, cas);
                }
        });

    /* -------------------- 创建 RTCVideo -------------------- */
    m_video = bytertc::createRTCVideo(m_appid.c_str(), m_handler.get(), nullptr);
    if (!m_video) return;

    m_video->setVideoSourceType(bytertc::kStreamIndexMain,
        bytertc::kVideoSourceTypeExternal);
    bytertc::VideoCanvas cas;
    cas.background_color = 0;
    cas.render_mode = bytertc::RenderMode::kRenderModeFit;
    cas.view = (void*)widget_local->winId();
    m_video->setLocalVideoCanvas(bytertc::kStreamIndexMain, cas);

    /* -------------------- 加入房间 -------------------- */
    m_room = m_video->createRTCRoom(m_roomid.c_str());
    if (!m_room) return;
    bytertc::UserInfo uinfo;
    uinfo.uid = m_uid.c_str();
    uinfo.extra_info = nullptr;
    bytertc::RTCRoomConfig conf;
    m_room->joinRoom(m_token.c_str(), uinfo, conf);
    m_room->setRTCRoomEventHandler(m_handler.get());

    push_check_timer_ = new QTimer(this);
    connect(push_check_timer_, &QTimer::timeout, this, [this]() {
        qDebug() << "[推流状态] 每秒推送帧数:" << frame_count_;
        frame_count_ = 0;
        });
    push_check_timer_->start(1000);


    last_frame_timer_.start();
    alarm_timer_ = new QTimer(this);
    connect(alarm_timer_, &QTimer::timeout, this, [this]() {
        // ---- 停帧检测 ----
        if (last_frame_timer_.elapsed() > 2000) {  // 2 秒无帧
            qWarning() << "已超过 2 秒未收到视频帧，疑似停帧！";
            last_frame_timer_.restart();
        }
        // ---- 黑帧检测 ----
        if (black_frame_count_ >= kBlackFrameConsecutive) {
            qWarning() << "连续" << black_frame_count_
                << "帧亮度过低，疑似黑帧！";
            black_frame_count_ = 0;
        }
        });
    alarm_timer_->start(500);

    ff_reader_.setFrameCallback([this](AVFrame* frame) { pushVideoFrame(frame); });
    ff_reader_.open("rtsp://admin:GU622723@192.168.1.64:554/Streaming/Channels/101");//rtsp://admin:GU622723@192.168.1.64:554/Streaming/Channels/101 //rtsp://s:123456@192.168.43.151:8554/streaming/live/1
    ff_reader_.start();
}


void RTCTest::pushVideoFrame(AVFrame* frame) {
    if (!frame || !m_video) return;

   
    cv::Mat bgr = I420ToBGR(frame);


    annotateRedBlue(bgr);

    BGRToI420(bgr, frame);

    const uint8_t* y_plane = frame->data[0];
    int            sample_pixels = std::min(100, frame->width * frame->height);
    int            sum_luma =
        std::accumulate(y_plane, y_plane + sample_pixels, 0);
    int avg_luma = sum_luma / sample_pixels;
    bool is_black = (avg_luma < kBlackFrameThreshold);
    black_frame_count_ = is_black ? black_frame_count_ + 1 : 0;

    /* ---------- 构建并推送 ---------- */
    bytertc::VideoFrameBuilder builder;
    builder.frame_type = bytertc::kVideoFrameTypeRawMemory;
    builder.pixel_fmt = bytertc::kVideoPixelFormatI420;
    builder.color_space = bytertc::kColorSpaceUnknown;
    builder.width = frame->width;
    builder.height = frame->height;
    builder.data[0] = frame->data[0];
    builder.data[1] = frame->data[1];
    builder.data[2] = frame->data[2];
    builder.linesize[0] = frame->linesize[0];
    builder.linesize[1] = frame->linesize[1];
    builder.linesize[2] = frame->linesize[2];
    builder.rotation = bytertc::kVideoRotation0;

    bytertc::IVideoFrame* video_frame = bytertc::buildVideoFrame(builder);
    if (video_frame) {
        m_video->pushExternalVideoFrame(video_frame);
        frame_count_++;
        last_frame_timer_.restart();
    }
}


RTCTest::~RTCTest() {
    ff_reader_.stop();
    ff_reader_.close();

    if (alarm_timer_)  alarm_timer_->stop();
    if (push_check_timer_) push_check_timer_->stop();

    if (m_room) {
        m_room->leaveRoom();
        m_room->destroy();
        m_room = nullptr;
    }
    if (m_video) {
        bytertc::destroyRTCVideo();
        m_video = nullptr;
    }
}


void EventHandler::onRoomStateChanged(const char* room_id, const char* uid,
    int state, const char* extra_info) {
    if (room_id && uid) {
        std::string str_extra_info = extra_info ? extra_info : "";
        emit sigRoomStateChanged(std::string(room_id), std::string(uid), state,
            str_extra_info);
    }
}

void EventHandler::onUserPublishStream(const char* uid,
    bytertc::MediaStreamType type) {
    if (uid) emit sigUserPublishStream(std::string(uid), type);
}

void EventHandler::onUserUnpublishStream(const char* uid,
    bytertc::MediaStreamType type,
    bytertc::StreamRemoveReason reason) {
    if (uid) emit sigUserUnpublishStream(std::string(uid), type, reason);
}
