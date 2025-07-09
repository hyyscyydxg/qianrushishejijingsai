import asyncio
import websockets
import json

# --- 您需要修改的部分 ---
# 请将这里的IP地址换成您遥控器的真实IP地址
REMOTE_CONTROLLER_IP = "192.168.229.160"  # 示例: "192.168.1.101"
pitch_all=0.0
yaw_all=0.0

# --------------------

async def send_command(command_data):
    """
    连接到WebSocket服务器，并发送一个指令的通用函数
    """
    uri = f"ws://{REMOTE_CONTROLLER_IP}:9876"

    try:
        # 设置一个较短的连接超时，防止程序卡住
        async with websockets.connect(uri, open_timeout=3) as websocket:
            await websocket.send(json.dumps(command_data))
            print(f"指令已发送: {json.dumps(command_data)}")

    except ConnectionRefusedError:
        print(f"错误：连接被拒绝。请确认遥控器IP({REMOTE_CONTROLLER_IP})正确且App正在运行。")
    except Exception as e:
        print(f"发生错误: {e}")


def main_loop():
    """
    主循环，接收用户输入并发送指令
    """
    global pitch_all
    global yaw_all
    print("\n--- DJI 云台网络控制器 (循环模式) ---")
    print("  'u' -> 云台向上转10度")
    print("  'd' -> 云台向下转10度")
    print("  'l' -> 云台向左转10度")
    print("  'a' -> 云台向右转10度")
    print("  'r' -> 云台回中 (需要您在UI上确认ResetGimbal的KeyName)")
    print("  'q' -> 退出程序")
    print("------------------------------------")

    while True:
        choice = input("请输入指令并按回车: ").lower()

        if choice == 'q':
            print("正在退出...")
            break

        command_to_send = None

        if choice == 'u' or choice == 'd':
            # 根据您的选择，设置pitch的值
            pitch_value = 10.0 if choice == 'u' else -10.0
            pitch_all+=pitch_value
            # 使用您提供的、正确的“扁平化”JSON格式
            gimbal_rotation_params = {
                "mode": 0,  # 0 代表相对角度
                "pitch": pitch_value,  # 俯仰角增量
                "roll": 0.0,
                "yaw": 0.0,
                "pitchIgnored": "false",
                "rollIgnored": "false",
                "yawIgnored": "false",
                "duration": 0.5,  # 旋转用时1.5秒
                "jointReferenceUsed": "false",
                "timeout": 0
            }

            command_to_send = {
                "keyName": "RotateByAngle",
                "params": gimbal_rotation_params
            }
        elif choice == 'l' or choice == 'a':
            # 根据您的选择，设置pitch的值
            yaw_value = 10.0 if choice == 'a' else -10.0
            yaw_all += yaw_value
            # 使用您提供的、正确的“扁平化”JSON格式
            gimbal_rotation_params = {
                "mode": 0,  # 0 代表相对角度
                "pitch": 0.0,  # 俯仰角增量
                "roll": 0.0,
                "yaw": yaw_value,
                "pitchIgnored": "false",
                "rollIgnored": "false",
                "yawIgnored": "false",
                "duration": 1,  # 旋转用时1.5秒
                "jointReferenceUsed": "false",
                "timeout": 0
            }

            command_to_send = {
                "keyName": "RotateByAngle",
                "params": gimbal_rotation_params
            }

        elif choice == 'r':
            gimbal_rotation_params = {
                "mode": 0,  # 0 代表相对角度
                "pitch": -pitch_all,  # 俯仰角增量
                "roll": 0.0,
                "yaw": -yaw_all,
                "pitchIgnored": "false",
                "rollIgnored": "false",
                "yawIgnored": "false",
                "duration": 0.5*((pitch_all+yaw_all)/10),  # 旋转用时1.5秒
                "jointReferenceUsed": "false",
                "timeout": 0
            }

            command_to_send = {
                "keyName": "RotateByAngle",
                "params": gimbal_rotation_params
            }
        else:
            print("无效指令，请重新输入。")
            continue

        # 使用asyncio.run来执行异步的发送函数
        if command_to_send:
            asyncio.run(send_command(command_to_send))


if __name__ == "__main__":
    main_loop()