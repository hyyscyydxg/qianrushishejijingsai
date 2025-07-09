#include "FfmpegRtspReader.h"
#include <iostream>

extern "C" {
#include <libavutil/time.h>
#include <libavutil/error.h>  // av_strerror
}

FfmpegRtspReader::FfmpegRtspReader() {
    avformat_network_init();
}

FfmpegRtspReader::~FfmpegRtspReader() {
    stop();
    close();
    avformat_network_deinit();
}

bool FfmpegRtspReader::open(const std::string& url) {
    int ret = 0;
    char errbuf[AV_ERROR_MAX_STRING_SIZE] = { 0 };

    fmt_ctx_ = avformat_alloc_context();
    AVDictionary* options = nullptr;
    av_dict_set(&options, "rtsp_transport", "tcp", 0);
    av_dict_set(&options, "stimeout", "5000000", 0); // 5秒超时

    if ((ret = avformat_open_input(&fmt_ctx_, url.c_str(), nullptr, &options)) < 0) {
        av_strerror(ret, errbuf, sizeof(errbuf));
        std::cerr << "Could not open input: " << errbuf << std::endl;
        return false;
    }

    if ((ret = avformat_find_stream_info(fmt_ctx_, nullptr)) < 0) {
        av_strerror(ret, errbuf, sizeof(errbuf));
        std::cerr << "Could not find stream info: " << errbuf << std::endl;
        return false;
    }

    for (unsigned int i = 0; i < fmt_ctx_->nb_streams; i++) {
        if (fmt_ctx_->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            video_stream_index_ = i;
            break;
        }
    }

    if (video_stream_index_ == -1) {
        std::cerr << "No video stream found." << std::endl;
        return false;
    }

    AVCodecParameters* codecpar = fmt_ctx_->streams[video_stream_index_]->codecpar;
    const AVCodec* decoder = avcodec_find_decoder(codecpar->codec_id);
    if (!decoder) {
        std::cerr << "Decoder not found." << std::endl;
        return false;
    }

    codec_ctx_ = avcodec_alloc_context3(decoder);
    if (!codec_ctx_) {
        std::cerr << "Could not allocate codec context." << std::endl;
        return false;
    }

    if ((ret = avcodec_parameters_to_context(codec_ctx_, codecpar)) < 0) {
        av_strerror(ret, errbuf, sizeof(errbuf));
        std::cerr << "Could not copy codec parameters: " << errbuf << std::endl;
        return false;
    }

    if ((ret = avcodec_open2(codec_ctx_, decoder, nullptr)) < 0) {
        av_strerror(ret, errbuf, sizeof(errbuf));
        std::cerr << "Could not open codec: " << errbuf << std::endl;
        return false;
    }

    frame_ = av_frame_alloc();
    pkt_ = av_packet_alloc();

    return true;
}

void FfmpegRtspReader::start() {
    running_ = true;
    decode_thread_ = std::thread(&FfmpegRtspReader::decodeLoop, this);
}

void FfmpegRtspReader::stop() {
    running_ = false;
    if (decode_thread_.joinable()) {
        decode_thread_.join();
    }
}

void FfmpegRtspReader::close() {
    if (pkt_) {
        av_packet_free(&pkt_);
        pkt_ = nullptr;
    }
    if (frame_) {
        av_frame_free(&frame_);
        frame_ = nullptr;
    }
    if (codec_ctx_) {
        avcodec_free_context(&codec_ctx_);
        codec_ctx_ = nullptr;
    }
    if (fmt_ctx_) {
        avformat_close_input(&fmt_ctx_);
        fmt_ctx_ = nullptr;
    }
}

void FfmpegRtspReader::setFrameCallback(FrameCallback cb) {
    frame_callback_ = cb;
}

void FfmpegRtspReader::decodeLoop() {
    char errbuf[AV_ERROR_MAX_STRING_SIZE] = { 0 };

    while (running_) {
        int ret = av_read_frame(fmt_ctx_, pkt_);
        if (ret < 0) {
            av_strerror(ret, errbuf, sizeof(errbuf));
            std::cerr << "Error reading frame: " << errbuf << std::endl;
            av_usleep(10 * 1000); // 等待10毫秒
            continue;
        }

        if (pkt_->stream_index == video_stream_index_) {
            ret = avcodec_send_packet(codec_ctx_, pkt_);
            if (ret < 0) {
                av_packet_unref(pkt_);
                continue;
            }

            while (ret >= 0) {
                ret = avcodec_receive_frame(codec_ctx_, frame_);
                if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                    break;

                if (ret >= 0 && frame_callback_) {
                    frame_callback_(frame_);
                }
            }
        }

        av_packet_unref(pkt_);
    }
}
