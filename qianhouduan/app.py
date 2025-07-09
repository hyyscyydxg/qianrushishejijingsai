import os, subprocess, signal, time, uuid, shutil, base64
from datetime import datetime
from flask import Flask, jsonify, request
from flask_cors import CORS
from volcenginesdkarkruntime import Ark
from upload_to_tos import upload_to_tos
from openai import OpenAI  # ← 确保顶部已导入
import tos
# ----------------- 配置参数 -----------------
RTSP_SOURCE  = "rtsp://admin:xxx3@192.168.1.64:554/Streaming/Channels/101"
STATIC_DIR   = "static"
HLS_M3U8     = f"{STATIC_DIR}/stream.m3u8"
RECORD_DIR   = "recordings"
RECORD_FILE  = os.path.join(RECORD_DIR, "recorded-full.ts")  # 原为 .mp4
CLIP_DIR     = "clips"
MAX_MB       = 52
ARK_LIMIT = 52 * 1024 * 1024  # Ark 最大支持 52 MB
TOS_BUCKET   = "video-test622723"
TOS_ACCESS_KEY = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
TOS_SECRET_KEY = "xxxxxxxxx=="
TOS_ENDPOINT   = "tos-cn-shanghai.volces.com"
TOS_PREFIX   = "clips/" + datetime.now().strftime("%Y%m%d")
SK_Key="sk-xxxxx"
ARK_API_KEY  = "5362e52d-xxxxxxxxxxxx-4a7d-8973-fadf2d897cb2"
os.environ["ARK_API_KEY"] = ARK_API_KEY

Prompt="总结视频内容并生成按每秒划分的日志，用'火源'代指红色物体（如卡片或色块），用'人员'代指蓝色物体（如卡片或色块），指出其在视频画面中的具体位置，不再描述'红色'蓝色',以“火源”“人员”作为输出描述的主体，也不要描述形状"
RTC_APP_ID   = "6863a1e7af71ec017450a73c"
RTC_APP_KEY  = "xxxxxxxxxxxxxxx"
RTC_PUSHER_EXE = os.path.abspath(r"rtc_pusher/RTCTest.exe")

# ----------------- 初始化 Flask -----------------
app = Flask(__name__, static_url_path="/static", static_folder=STATIC_DIR)
CORS(app)

# ----------------- 推流与录像进程 -----------------
proc_hls = proc_record = proc_rtc = None

def start_ffmpeg_hls():
    global proc_hls
    if proc_hls: return
    os.makedirs(STATIC_DIR, exist_ok=True)
    cmd = [
        "ffmpeg", "-rtsp_transport", "tcp", "-i", RTSP_SOURCE,
        "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
        "-f", "hls", "-hls_time", "1", "-hls_list_size", "3",
        "-hls_flags", "delete_segments+append_list+omit_endlist",
        HLS_M3U8
    ]
    proc_hls = subprocess.Popen(cmd)
    print(f"[HLS] PID={proc_hls.pid}")

# ----------------- 录像：按文件大小直接切到 clips/ -----------------
def start_ffmpeg_record():
    global proc_record
    if proc_record:
        return
    os.makedirs(RECORD_DIR, exist_ok=True)
    if os.path.exists(RECORD_FILE):
        os.remove(RECORD_FILE)
    cmd = [
        "ffmpeg", "-rtsp_transport", "tcp", "-i", RTSP_SOURCE,
        "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
        "-c:a", "aac",
        "-f", "mpegts",
        RECORD_FILE
    ]
    proc_record = subprocess.Popen(cmd)
    print(f"[Record] PID={proc_record.pid}")


def start_rtc_pusher():
    global proc_rtc
    print("→ 正在尝试启动 RTCTest.exe")
    if not os.path.exists(RTC_PUSHER_EXE):
        print("[RTC] RTCTest.exe 路径不存在！")
        return
    if proc_rtc is not None and proc_rtc.poll() is None:
        print("[RTC] 推流器已运行中，跳过启动")
        return
    proc_rtc = subprocess.Popen([RTC_PUSHER_EXE])
    print(f"[RTC] 启动成功，PID = {proc_rtc.pid}")

def stop_process(p):
    if p and p.poll() is None:
        print(f"[🛑] 发送 SIGTERM: PID={p.pid}")
        p.send_signal(signal.SIGTERM)
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            print(f"[⚠️] 强制 kill: PID={p.pid}")
            p.kill()

def stop_all():
    print("[🧹] 停止所有子进程")
    for p in (proc_rtc, proc_record, proc_hls):
        stop_process(p)

# ----------------- 信号处理 -----------------
def handle_exit(signum=None, frame=None):
    print(f"[🚪] 收到退出信号 {signum}，正在清理...")
    stop_all()
    os._exit(0)

signal.signal(signal.SIGINT, handle_exit)
signal.signal(signal.SIGTERM, handle_exit)

import atexit
atexit.register(stop_all)

# ----------------- Ark 日志生成 -----------------

def ark_log(video_url: str) -> str:
    try:
        print(f"[Ark] 正在调用 Ark API 分析视频: {video_url}")
        client = Ark(api_key=ARK_API_KEY)
        resp = client.chat.completions.create(
            model="doubao-seed-1.6-250615",
            messages=[{
                "role": "user",
                "content": [
                    {"type": "video_url", "video_url": {"url": video_url, "fps": 2}},
                    {"type": "text", "text": Prompt}
                ]
            }]
        )
        print("[Ark] Ark 返回成功")
        return resp.choices[0].message.content.strip()
    except Exception as e:
        print("[Ark] 调用失败:", str(e))
        return "【Ark 调用失败】"


# ----------------- 接口路由 -----------------
@app.route("/api/start_rtc", methods=["POST"])
def api_start_rtc():
    start_rtc_pusher()
    return jsonify({"message": "RTC 推流进程启动中"})


@app.route("/split_and_log", methods=["POST"])
def split_and_log():
    try:
        # --- 目录准备 ---
        os.makedirs(CLIP_DIR, exist_ok=True)
        for f in os.listdir(CLIP_DIR):
            os.remove(os.path.join(CLIP_DIR, f))

        # --- 录制文件检查 ---
        if not os.path.exists(RECORD_FILE):
            return {"status": "error", "message": "录制文件暂未生成"}, 400
        print("使用的录制文件路径:", RECORD_FILE)

        # === 生成快照（复制当前已写入部分） ===
        snapshot_path = os.path.join(CLIP_DIR, "snapshot.ts")
        shutil.copy2(RECORD_FILE, snapshot_path)
        snapshot_size_mb = os.path.getsize(snapshot_path) / 1024 / 1024
        print(f"[Snapshot] 复制完成 → {snapshot_path}")
        print(f"快照文件大小：{snapshot_size_mb:.2f} MB")

        # === 使用 ffprobe 估算平均码率，按时间切段控制大小 ===
        probe_cmd = [
            "ffprobe", "-v", "error", "-select_streams", "v:0",
            "-show_entries", "format=duration",
            "-of", "default=noprint_wrappers=1:nokey=1",
            snapshot_path
        ]
        probe_result = subprocess.run(probe_cmd, capture_output=True, text=True)
        duration = float(probe_result.stdout.strip())
        snapshot_bytes = os.path.getsize(snapshot_path)
        avg_bps = snapshot_bytes / duration
        max_bytes = MAX_MB * 1024 * 1024
        est_sec = int((max_bytes * 0.75) / avg_bps)  # 加安全系数
        if est_sec < 2:
            est_sec = 2
        print(f"[估算切割] 每秒约 {avg_bps:.1f} B/s → 每段最长 {est_sec}s")

        # === 使用 ffmpeg 按时间切段 ===
        ts_tmpl = os.path.join(CLIP_DIR, "seg%03d.ts")
        cmd_split = [
        "ffmpeg", "-y", "-i", snapshot_path,
        "-c", "copy",
        "-f", "segment",
        "-segment_time", str(est_sec),
        "-fs", str(ARK_LIMIT),          # 52 MB 上限
        "-reset_timestamps", "1",
        ts_tmpl
        ]

        result = subprocess.run(cmd_split, capture_output=True, text=True)
        if result.returncode != 0:
            print("切割失败，stderr")
            print(result.stderr)
            os.remove(snapshot_path)
            return {
                "status": "error",
                "message": "ffmpeg 切割失败",
                "detail": result.stderr
            }, 500

        clips = []
        for fname in sorted(os.listdir(CLIP_DIR)):
            if not fname.endswith(".ts") or fname == "snapshot.ts":
                continue
            ts_path = os.path.join(CLIP_DIR, fname)
            mp4_name = fname.replace(".ts", ".mp4")
            mp4_path = os.path.join(CLIP_DIR, mp4_name)

            cmd_mp4 = ["ffmpeg", "-y", "-fflags", "+genpts", "-i", ts_path, "-c", "copy", mp4_path]
            res_mp4 = subprocess.run(cmd_mp4, capture_output=True, text=True)
            if res_mp4.returncode != 0:
                print(f"ts→mp4 失败 ({fname})，stderr ↓↓↓")
                print(res_mp4.stderr)
                continue

            public_url = upload_to_tos(
                local_path=mp4_path,
                bucket=TOS_BUCKET,
                object_prefix=f"{TOS_PREFIX}/{uuid.uuid4()}.mp4",
                access_key=TOS_ACCESS_KEY,
                secret_key=TOS_SECRET_KEY,
                endpoint=TOS_ENDPOINT,
                region="cn-shanghai"
            )

            print(f"[上传成功] {mp4_name} → {public_url}")
            log_result = ark_log(public_url)
            print("[Ark 日志内容] ")
            print(log_result)
            clips.append({
                "file": mp4_name,
                "url": public_url,
                "log": log_result
            })

        # --- 清理快照 ---
        os.remove(snapshot_path)

        try:
            vei_client = OpenAI(
                base_url="https://ai-gateway.vei.volces.com/v1",
                api_key=SK_Key
            )
            all_logs = "\n\n".join([c["log"] for c in clips if c.get("log")])
            summary_resp = vei_client.chat.completions.create(
                model="7522777388595003438",
                messages=[
                    {"role": "system", "content": "请你整合以下多个分段视频的日志，按时刻总结整体监控情况。"},
                    {"role": "user", "content": all_logs}
                ]
            )
            summary = summary_resp.choices[0].message.content.strip()
            print("[VEI 总结] ")
            print(summary)
            summary_key = f"summary_logs/{uuid.uuid4()}.txt"
            try:
                tos_client = tos.TosClientV2(
                    TOS_ACCESS_KEY, TOS_SECRET_KEY,
                    TOS_ENDPOINT, "cn-shanghai"
                )
                tos_client.put_object(
                    TOS_BUCKET,
                    summary_key,
                    content=summary.encode("utf-8")
                )
                summary_url = f"https://{TOS_BUCKET}.{TOS_ENDPOINT}/{summary_key}"
                print(f"[日志总结已上传] {summary_url}")
            except Exception as e:
                print(f"[日志总结上传失败] {e}")
                summary_url = None

        except Exception as e:
            summary = "【VEI 总结失败】" + str(e)
            summary_url = None
            print("[VEI 调用失败]", e)

        return {"status": "ok", "summary": summary}

    except Exception as e:
        import traceback
        print("Exception occurred:\n", traceback.format_exc())
        return {"status": "error", "message": str(e)}, 500



@app.route("/trigger_log", methods=["POST"])
def trigger_log():
    return split_and_log()

@app.route("/health", methods=["GET"])
def health():
    return {"status": "running"}

# ----------------- 启动入口 -----------------
if __name__ == "__main__":
    try:
        start_ffmpeg_hls()
        start_ffmpeg_record()
        start_rtc_pusher()
        try:
            app.run(host="0.0.0.0", port=5000)
        except Exception as e:
            print("[Flask 异常]", e)
    finally:
        stop_all()
