from miservice.cli import micli
from dotenv import load_dotenv
from config import get_env
from io import StringIO
from contextlib import redirect_stdout
import json
load_dotenv()

def get_devices():
    # 捕获标准输出
    f = StringIO()
    with redirect_stdout(f):
        micli('list')
    output = f.getvalue()
    print("正在遍历设备列表")
    # 遍历设备列表
    speaker_device = 0
    for device in json.loads(output):
        if "xiaomi.wifispeaker" in device["model"]:
            name = device["name"]
            model = device["model"].replace("xiaomi.wifispeaker.", "").capitalize()
            did = device["did"]
            # 注释
            comment = f"# 设备名称：{name}，型号：{model}，DID：{did}\n"
            # 写入环境变量
            with open(".env", "a") as f:
                f.write(comment)
                print(comment)
                if speaker_device == 0:
                    f.write(f'export MI_DID="{did}"\n')
                    f.write(f'export MI_MODEL="{model}"\n')
                    speaker_device += 1
                else:
                    f.write(f'export MI_DID{speaker_device+1}="{did}"\n')
                    f.write(f'export MI_MODEL{speaker_device+1}="{model}"\n')
                    speaker_device += 1
    if speaker_device == 0:
        print("查找结束，未找到设备")
        return
    elif speaker_device >= 2:
        print("查找结束，找到多个设备，默认监听第一个设备")
    get_env()

def set_volume(volume:int):
    volume = max(5, min(volume, 100))
    if volume == 5:
        micli("2-2=#true")
    else:
        micli("2-1=#"+str(volume))
if __name__ == "__main__":
    get_devices()
    # micli("list")
    # micli("spec","spec xiaomi.wifispeaker.lx06")
    # micli("spec","spec xiaomi.wifispeaker.lx06") # 查看设备信息
    # micli("2-1") # 查询当前音量
    # micli("2-2=#false") # 取消静音
    # micli("2-2=#true") # 设置静音
    # micli("4-1=true") # 设置麦克风关闭
    # micli("5-1","你好") # 播放回答
