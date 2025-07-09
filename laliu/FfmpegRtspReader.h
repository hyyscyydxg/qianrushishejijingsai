#pragma once

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

#include <string>
#include <thread>
#include <atomic>
#include <functional>

class FfmpegRtspReader {
public:
    using FrameCallback = std::function<void(AVFrame* frame)>;

    FfmpegRtspReader();
    ~FfmpegRtspReader();

    bool open(const std::string& url);
    void setFrameCallback(FrameCallback cb);
    void start();
    void stop();
    void close();

private:
    void decodeLoop();

private:
    AVFormatContext* fmt_ctx_ = nullptr;
    AVCodecContext* codec_ctx_ = nullptr;
    AVFrame* frame_ = nullptr;
    AVPacket* pkt_ = nullptr;
    int video_stream_index_ = -1;
    std::thread decode_thread_;
    std::atomic<bool> running_{ false };
    FrameCallback frame_callback_;
};
