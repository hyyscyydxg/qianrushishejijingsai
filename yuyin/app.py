import os
from dotenv import load_dotenv
import io
import azure.cognitiveservices.speech as speechsdk
from openai import OpenAI
import time
import datetime
import threading
import json, ast

import requests
from io import BytesIO
import tempfile
import numpy as np
# import pyaudio

# ➡️ 新增：ROS 2 依赖
import rclpy
from std_msgs.msg import String

# 导入httpx和httpx_socks
import httpx
from httpx_socks import SyncProxyTransport

# 加载环境变量
load_dotenv("voice1.env")

# 设置代理地址
proxy_url = "socks5://127.0.0.1:7898"  # 你可以更改为你实际使用的代理地址
transport = SyncProxyTransport.from_url(proxy_url)

# 在这里创建 httpx 客户端，并且不会关闭它
client = httpx.Client(transport=transport)

# 使用 client 初始化 OpenAI 客户端
client_openai = OpenAI(api_key=os.environ["api_key"],
                       base_url=os.environ["base_url"],
                       http_client=client)

# Azure 设置
Azure_speech_key = os.environ["Azure_speech_key"]
Azure_speech_region = os.environ["Azure_speech_region"]
Azure_speech_speaker = os.environ["Azure_speech_speaker"]
WakeupWord = os.environ["WakeupWord"]
WakeupModelFile = os.environ["WakeupModelFile"]

messages = []

# Azure语音到文本和文本到语音设置
speech_key = Azure_speech_key
service_region = Azure_speech_region
speech_config = speechsdk.SpeechConfig(subscription=speech_key, region=service_region)
speech_config.speech_synthesis_language = "zh-CN"
speech_config.speech_recognition_language = "zh-CN"
lang = "zh-CN"
speech_config.speech_synthesis_voice_name = Azure_speech_speaker
speech_synthesizer = speechsdk.SpeechSynthesizer(speech_config=speech_config)
connection = speechsdk.Connection.from_speech_synthesizer(speech_synthesizer)
connection.open(True)
model = speechsdk.KeywordRecognitionModel(WakeupModelFile)
keyword = WakeupWord
audio_config = speechsdk.audio.AudioConfig(use_default_microphone=True)
auto_detect_source_language_config = speechsdk.languageconfig.AutoDetectSourceLanguageConfig(
    languages=["ja-JP", "zh-CN"])
speech_recognizer = speechsdk.SpeechRecognizer(
    speech_config=speech_config,
    audio_config=audio_config,
    auto_detect_source_language_config=auto_detect_source_language_config)

unknownCount = 0
sysmesg = {"role": "system", "content": os.environ["sysprompt_zh-CN"]}
tts_sentence_end = [".", "!", "?", ";", "。", "！", "？", "；", "\n"]

# 新增的导航指令
NAVIGATION_PROMPT = """你是一个专业的语音助手。在本次回答中，请严格遵循以下规则：如果用户的提问包含明确的导航或带路意图（例如“带我去某个地方”、“导航到某个地点”、“我要去哪里”等），你的回答格式必须是：“好的，我将带您到[目标地点]”，不要添加任何其他多余的解释或文字。例如，如果用户说“带我去人民广场”，你就必须只回答“好的，我将带您到人民广场”。对于所有其他类型的问题，请像平常一样正常回答。"""

isListenning = False

# ➡️ 新增：ROS 全局对象
ros_node = None
ros_publisher = None
ros_publisher_distinguish = None

def display_text(s):
    print(s)


def speech_to_text():
    global unknownCount
    global lang, isListenning
    print("Please say...")
    result = speech_recognizer.recognize_once_async().get()
    if result.reason == speechsdk.ResultReason.RecognizedSpeech:
        unknownCount = 0
        isListenning = False
        return result.text
    elif result.reason == speechsdk.ResultReason.NoMatch:
        isListenning = False
        unknownCount += 1
        error = os.environ["sorry_" + lang]
        text_to_speech(error)
        return '...'
    elif result.reason == speechsdk.ResultReason.Canceled:
        isListenning = False
        return "speech recognizer canceled."


def getVoiceSpeed():
    return 17


def text_to_speech(text, _lang=None):
    global lang
    try:
        result = buildSpeech(text).get()
        if result.reason == speechsdk.ResultReason.SynthesizingAudioCompleted:
            print("Text-to-speech conversion successful.")
            return "Done."
        else:
            print(f"Error synthesizing audio: {result}")
            return "Failed."
    except Exception as ex:
        print(f"Error synthesizing audio: {ex}")
        return "Error occured!"


def buildSpeech(text, _lang=None):
    voice_lang = lang
    voice_name = "zh-CN-XiaoxiaoMultilingualNeural"
    ssml_text = f'''
        <speak xmlns="http://www.w3.org/2001/10/synthesis" xmlns:mstts="http://www.w3.org/2001/mstts" xmlns:emo="http://www.w3.org/2009/10/emotionml" version="1.0" xml:lang="{lang}"><voice name="{voice_name}"><lang xml:lang="{voice_lang}"><prosody rate="{getVoiceSpeed()}%">{text.replace('*', ' ')}</prosody></lang></voice></speak>
    '''
    print(f"{voice_name} {voice_lang}!")
    return speech_synthesizer.speak_ssml_async(ssml_text)


def generate_text(prompt):
    global ros_publisher_distinguish
    global messages, sysmesg

    # 1. 将用户提问加入长期历史
    messages.append({"role": "user", "content": prompt})

    # 2. 构造本次 API 调用的临时消息列表
    temp_messages_for_api = []
    temp_messages_for_api.append(sysmesg)
    if len(messages) > 1:
        temp_messages_for_api.extend(messages[-7:-1])
    temp_messages_for_api.append({"role": "system", "content": NAVIGATION_PROMPT})
    temp_messages_for_api.append({"role": "user", "content": prompt})

    collected_messages = []
    last_tts_request = None
    split = True
    result = ''

    response_gen = client_openai.chat.completions.create(
        model="deepseek-chat",
        messages=temp_messages_for_api,
        stream=True
    )

    for chunk in response_gen:
        if chunk:
            chunk_message = chunk.choices[0].delta.content
            if chunk_message is not None:
                collected_messages.append(chunk_message)
                result += chunk_message

                if chunk_message in tts_sentence_end and split:
                    text = ''.join(collected_messages).strip()
                    if len(text) > 20:
                        split = False
                    elif len(text) < 6:
                        continue
                    if text != '':
                        print(f"Speech synthesized to speaker for: {text}")
                        last_tts_request = buildSpeech(text)
                        collected_messages.clear()

    # 3. 将 AI 完整回答加入长期历史
    messages.append({"role": "assistant", "content": result})

    if len(collected_messages) > 0:
        text = ''.join(collected_messages).strip()
        if text != '':
            print(f"Speech synthesized to speaker for: {text}")
            last_tts_request = buildSpeech(text)
            collected_messages.clear()

    if last_tts_request:
        last_tts_request.get()

    if ros_publisher  is not None:
        ros_msg = String()
        ros_msg.data = result
        ros_publisher.publish(ros_msg)

    if ros_publisher_distinguish and any(c in result for c in ['好的，我将带您到a点', '好的，我将带您到A点']):
       	msg = String()
        msg.data = 'A'
        ros_publisher_distinguish.publish(msg)
    elif ros_publisher_distinguish and any(c in result for c in ['好的，我将带您到b点', '好的，我将带您到B点']):
       	msg = String()
        msg.data = 'B'
        ros_publisher_distinguish.publish(msg)
    elif ros_publisher_distinguish and any(c in result for c in ['好的，我将带您到c点', '好的，我将带您到C点']):
       	msg = String()
        msg.data = 'C'
        ros_publisher_distinguish.publish(msg)
        
    return result


# 识别到关键词时的回调函数
def recognized_cb(evt):
    result = evt.result
    if result.reason == speechsdk.ResultReason.RecognizedKeyword:
        print("RECOGNIZED KEYWORD: {}".format(result.text))


# 识别取消时的回调函数
def canceled_cb(evt):
    result = evt.result
    if result.reason == speechsdk.ResultReason.Canceled:
        print('CANCELED: {}'.format(result.cancellation_details.reason))


def start_recognition():
    global unknownCount, isListenning
    while True:
        keyword_recognizer = speechsdk.KeywordRecognizer()
        keyword_recognizer.recognized.connect(recognized_cb)
        keyword_recognizer.canceled.connect(canceled_cb)
        first = os.environ["welcome_" + lang]
        display_text(first)
        text_to_speech(first)
        isListenning = True
        result_future = keyword_recognizer.recognize_once_async(model)
        while True:
            result = result_future.get()
            if result.reason == speechsdk.ResultReason.RecognizedKeyword:
                print("Keyword recognized")
                isListenning = False
                break
            time.sleep(0.1)

        display_text("很高兴为您服务，我在听请讲。")
        text_to_speech("很高兴为您服务，我在听请讲。")

        while unknownCount < 2:
            isListenning = True
            user_input = speech_to_text()
            print(f"You: {user_input}")
            display_text(f"You: {user_input}")
            response = generate_text(user_input)

        bye_text = os.environ["bye_" + lang]
        display_text(bye_text)
        text_to_speech(bye_text)

        unknownCount = 0
        time.sleep(0.1)


if __name__ == "__main__":
    # ➡️ 新增：ROS 2 初始化
    rclpy.init()
    ros_node = rclpy.create_node("voice_assistant_node")
    ros_publisher = ros_node.create_publisher(String,"/voice_assistant/response", 10)
    ros_publisher_distinguish = ros_node.create_publisher(String,"/voice_assistant/distinguish", 10)
    try:
        start_recognition()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown() 
