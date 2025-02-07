from config import get_env, get_mi_device
get_env()
from miservice import MiAccount, MiNAService
from aiohttp import ClientSession
import os, json, asyncio, time
from pathlib import Path
from http.cookies import SimpleCookie
from requests.utils import cookiejar_from_dict
from _mihome import get_devices, set_volume
from chat import chat
from miservice.cli import run_command


def get_last_timestamp_and_record(data):
    if "data" in data:
        d = data.get("data")
        records = json.loads(d).get("records")
        if not records:
            return 0, None
        last_record = records[0]
        timestamp = last_record.get("time")
        return timestamp, last_record["query"]
    else:
        return 0, None


def parse_cookie_string(cookie_string):
    cookie = SimpleCookie()
    cookie.load(cookie_string)
    cookies_dict = {}
    cookiejar = None
    for k, m in cookie.items():
        cookies_dict[k] = m.value
        cookiejar = cookiejar_from_dict(cookies_dict, cookiejar=None, overwrite=True)
    return cookiejar

Iservice = None

async def main():
    global Iservice
    cookies = []
    # 用户配置
    device_list = get_mi_device()
    hardware = os.getenv("MI_MODEL")
    user_id = os.getenv("MI_USER")
    password = os.getenv("MI_PASSWORD")
    token_path = os.path.join(str(Path.home()), ".mi.token")
    LATEST_ASK_API = "https://userprofile.mina.mi.com/device_profile/v2/conversation?source=dialogu&hardware={hardware}&timestamp={timestamp}&limit=2"
    COOKIE_TEMPLATE = (
        "deviceId={device_id}; serviceToken={service_token}; userId={user_id}"
    )

    async with ClientSession() as session:
        account = MiAccount(session, user_id, password, token_path)
        await account.login("micoapi")
        Iservice = MiNAService(account)
        deviceresult = await Iservice.device_list()
        if not isinstance(deviceresult, str):
            for h in deviceresult:
                if h.get("miotDID", "") in device_list:
                    deviceid = h.get("deviceID")
                with open(token_path) as f:
                    user_data = json.loads(f.read())
                service_token = user_data.get("micoapi")[1]
                cookiestr = COOKIE_TEMPLATE.format(
                    device_id=deviceid,
                    service_token=service_token,
                    user_id=user_id,
                )
                cookie = parse_cookie_string(cookiestr)
                cookies.append(cookie)

        chat_history = []
        while True:
            # 轮询多个设备的对话记录
            for cookie in cookies:
                try:
                    async with session.get(
                        LATEST_ASK_API.format(
                            hardware=hardware, timestamp=str(int(time.time() * 1000))
                        ),
                        cookies=parse_cookie_string(cookie),
                        timeout=10  # 增加超时时间
                    ) as response:
                        chat_msg = await response.json()
                        new_timestamp, last_record = get_last_timestamp_and_record(chat_msg)
                        if len(chat_history) <= 1:
                            chat_history.append((new_timestamp, last_record))
                        elif (new_timestamp, last_record) not in chat_history:
                            print("Q:",last_record)
                            chat_history.append((new_timestamp, last_record))
                            call_agent = chat(last_record)
                            if call_agent[1]:
                                print("A:",call_agent[0])
                                # 将回答代入接口，播放回答
                                await run_command("5-1", call_agent[0])
                except asyncio.TimeoutError:
                    print("请求超时，正在重试...")
                    continue  # 继续下一个循环
                # 等待1秒
                await asyncio.sleep(1)

if __name__ == "__main__":
    if not os.getenv("MI_DID") or not os.getenv("MI_MODEL"):
        get_devices()
    asyncio.run(main())
