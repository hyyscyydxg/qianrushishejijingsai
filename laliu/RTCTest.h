#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_RTCTest.h"
#include "bytertc_video.h"
#include "bytertc_room.h"
#include "bytertc_room_event_handler.h"
#include "bytertc_video_event_handler.h"
#include <QHBoxLayout>
#include <QMessageBox>
#include <QDebug>
#include <QMetaType>
#include <memory>
#include <QTimer>          // ✅ 新增
#include <QElapsedTimer>   // ✅ 新增
#include "FfmpegRtspReader.h"

Q_DECLARE_METATYPE(std::string)
Q_DECLARE_METATYPE(bytertc::MediaStreamType)
Q_DECLARE_METATYPE(bytertc::StreamRemoveReason)

class EventHandler : public QObject,
    public bytertc::IRTCVideoEventHandler,
    public bytertc::IRTCRoomEventHandler {
    Q_OBJECT
public:
    void onRoomStateChanged(const char* room_id, const char* uid,
        int state, const char* extra_info) override;
    void onUserPublishStream(const char* uid,
        bytertc::MediaStreamType type) override;
    void onUserUnpublishStream(const char* uid,
        bytertc::MediaStreamType type,
        bytertc::StreamRemoveReason reason) override;

signals:
    void sigRoomStateChanged(std::string roomid, std::string uid,
        int state, std::string extra_info);
    void sigUserPublishStream(std::string uid, bytertc::MediaStreamType type);
    void sigUserUnpublishStream(std::string uid, bytertc::MediaStreamType type,
        bytertc::StreamRemoveReason reason);
};

class RTCTest : public QMainWindow {
    Q_OBJECT

public:
    RTCTest(QWidget* parent = nullptr);
    ~RTCTest();

private:
    Ui::RTCTestClass ui;
    QWidget* widget_local = nullptr;
    QWidget* widget_remote = nullptr;

    bytertc::IRTCVideo* m_video = nullptr;
    bytertc::IRTCRoom* m_room = nullptr;

    std::string m_roomid = "040514";
    std::string m_uid = "yayun";
    std::string m_appid = "6863a1e7af71ec017450a73c";
    std::string m_token = "0016863a1e7af71ec017450a73cQQCzQgIE5v1kaGY4bmgGADA0MDUxNAUAeWF5dW4GAAAAZjhuaAEAZjhuaAIAZjhuaAMAZjhuaAQAZjhuaAUAZjhuaCAA6qOoAq5NKsJBPsRHgxi8iKZ4uiryAefQH2UAjCs9ISU=";

    std::shared_ptr<EventHandler> m_handler;
    FfmpegRtspReader             ff_reader_;

    void pushVideoFrame(AVFrame* frame);

    /* ---------- 监控与告警成员 ---------- */
    int      frame_count_ = 0;
    QTimer* push_check_timer_ = nullptr;
    QElapsedTimer last_frame_timer_;
    QTimer* alarm_timer_ = nullptr;
    int      black_frame_count_ = 0;

    static constexpr int kBlackFrameThreshold = 10; // Y 分量均值阈值
    static constexpr int kBlackFrameConsecutive = 30; // 连续黑帧触发阈值
};
