import asyncio
import logging
import json
import os
import sys
import tempfile
import random
from pathlib import Path

from miservice import (
    MiAccount,
    MiNAService,
    MiIOService,
    miio_command,
    miio_command_help,
)

from aiohttp import ClientSession
from mutagen.mp3 import MP3
from rich import print

def usage():
    print("MiService %s - XiaoMi Cloud Service\n")
    print("Usage: The following variables must be set:")
    print("           export MI_USER=<Username>")
    print("           export MI_PASS=<Password>")
    print("           export MI_DID=<Device ID|Name>\n")
    print(miio_command_help(prefix="micli" + " "))


def find_device_id(hardware_data, mi_did):
    for h in hardware_data:
        if h.get("miotDID", "") == str(mi_did):
            return h.get("deviceID")
        else:
            continue
    else:
        raise Exception(f"we have no mi_did: please use `micli mina` to check")


async def _get_duration(url:str, start=0, end=500):
    url = url.strip()
    # drop url params
    url_base = url.split("?")[0]
    if not url_base.endswith(".mp3"):
        async with ClientSession() as session:
            async with session.get(
                url,
                allow_redirects=True,
                headers={
                    "User-Agent": "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_10_1) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/39.0.2171.95 Safari/537.36"
                },
            ) as response:
                url = response.url

    headers = {"Range": f"bytes={start}-{end}"}
    async with ClientSession() as session:
        async with session.get(url, headers=headers) as response:
            array_buffer = await response.read()
    with tempfile.NamedTemporaryFile() as tmp:
        tmp.write(array_buffer)
        try:
            m = MP3(tmp)
        except:
            headers = {"Range": f"bytes={0}-{1000}"}
            async with session.get(url, headers=headers) as response:
                array_buffer = await response.read()
        with tempfile.NamedTemporaryFile() as tmp2:
            tmp2.write(array_buffer)
            m = MP3(tmp2)
    return m.info.length


async def get_suno_playlist(is_random=False):
    suno_playlist_url = "https://studio-api.suno.ai/api/playlist/1190bf92-10dc-4ce5-968a-7a377f37f984/?page=1"
    song_play_dict = {}
    async with ClientSession() as session:
        async with session.get(suno_playlist_url) as response:
            data = await response.json()
            for d in data["playlist_clips"]:
                clip = d.get("clip")
                if clip:
                    if clip.get("audio_url"):
                        song_play_dict[clip["audio_url"]] = clip["title"]
            return song_play_dict


# TODO support more
device_id_list = []


async def miservice_stop(device_id):
    """
    for ctrl + c exit
    """
    async with ClientSession() as session:
        env = os.environ
        account = MiAccount(
            session,
            env.get("MI_USER"),
            env.get("MI_PASS"),
            os.path.join(str(Path.home()), ".mi.token"),
        )
        mina_service = MiNAService(account)
        await mina_service.player_stop(device_id)
    print("Stop")

async def run_command(*args):
    """
    参数示例:
    - main("play", "http://example.com/music.mp3")
    - main("message", "你好世界")
    - main("stop")
    """
    try:
        async with ClientSession() as session:
            env = os.environ
            account = MiAccount(
                session,
                os.getenv("MI_USER"),
                os.getenv("MI_PASS"),
                os.path.join(str(Path.home()), ".mi.token"),
            )
            result = ""
            mina_service = MiNAService(account)
            # 将参数列表转换为字符串
            args_str = " ".join(args)
            if args[0].strip() in [
                "message",
                "play",
                "mina",
                "pause",
                "stop",
                "loop",
                "play_list",
                "suno",
                "suno_random",
            ]:
                arg = args[0].strip()
                result = await mina_service.device_list()
                if not env.get("MI_DID"):
                    raise Exception("Please export MI_DID in your env")
                device_id = find_device_id(result, env.get("MI_DID", ""))
                device_id_list.append(device_id)
                
                if len(args) == 1:
                    if args[0] in ["pause", "stop"]:
                        await mina_service.player_stop(device_id)
                    elif args[0] == "mina" and len(result) > 0:
                        print(result[0])
                    elif "suno" in args_list[0]:
                        song_dict = await get_suno_playlist()
                        print(song_dict)
                        song_urls = list(song_dict.keys())
                        if args_list[0] == "suno_random":
                            random.shuffle(song_urls)
                            print("Will play suno trending list randomly")
                        else:
                            print("Will play suno trending list")
                        for song_url in song_urls:
                            title = song_dict[song_url]
                            print(f"Will play {song_url.strip()} title {title}")
                            duration = await _get_duration(song_url)
                            await mina_service.play_by_url(device_id, song_url.strip())
                            await asyncio.sleep(duration)
                        await mina_service.player_stop(device_id)
                    else:
                        print("Please provide a play URL")
                    return
                
                if arg == "loop":
                    url = args[1]
                    await mina_service.play_by_url(device_id, url)
                    await mina_service.player_set_loop(device_id, 0)
                    return
                elif arg == "play":
                    await mina_service.play_by_url(device_id, args[1])
                    await mina_service.player_set_loop(device_id, 1)
                    return
                elif arg == "play_list":
                    try:
                        await mina_service.player_set_loop(device_id, 1)
                        file_name = args_list[1]
                        with open(file_name, encoding="utf8") as f:
                            lines = f.readlines()
                            for line in lines:
                                print(f"Will play {line.strip()}")
                                duration = await _get_duration(line)
                                await mina_service.play_by_url(device_id, line.strip())
                                await asyncio.sleep(duration)
                        await mina_service.player_stop(device_id)
                    except Exception as e:
                        print(e)
                        return
                elif arg == "message":
                    await mina_service.text_to_speech(device_id, args[1])
                    return
            else:
                service = MiIOService(account)
                result = await miio_command(
                    service, env.get("MI_DID"), args_str, "micli "
                )
            if not isinstance(result, str):
                result = json.dumps(result, indent=2, ensure_ascii=False)
    except Exception as e:
        result = e
    print(result)

def micli(*args, log_level=logging.WARNING):
    """
    函数调用示例:
    - micli("play", "http://example.com/music.mp3")
    - micli("message", "你好世界")
    - micli("stop")
    
    :param args: 命令和参数
    :param log_level: 日志级别，默认为 WARNING
    """
    if args:
        if log_level != logging.NOTSET:
            _LOGGER = logging.getLogger("miservice")
            _LOGGER.setLevel(log_level)
            _LOGGER.addHandler(logging.StreamHandler())
        try:
            if asyncio.get_event_loop().is_running():
                # 在当前事件循环中创建任务并等待执行完成
                loop = asyncio.get_event_loop()
                task = loop.create_task(run_command(*args))
                loop.run_until_complete(task)
            else:
                asyncio.run(run_command(*args))
        except (KeyboardInterrupt, asyncio.exceptions.CancelledError) as e:
            device_id = device_id_list[0]
            asyncio.run(miservice_stop(device_id))
            print(str(e))
    else:
        usage()


if __name__ == "__main__":
    micli()
