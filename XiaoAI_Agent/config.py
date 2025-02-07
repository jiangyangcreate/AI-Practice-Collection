import os
import dotenv
from collections import defaultdict  # 添加导入


def get_env():
    # 判断是否存在.env文件
    if not os.path.exists(".env"):
        # 创建.env文件
        print("请先在.env文件中设置账号和密码")
        with open(".env", "w") as f:
            f.write(
                """export PETKIT_NAME=""
export PETKIT_PASSWORD=""
export MI_USER = ""
export MI_PASSWORD = ""
"""
            )
        exit()
    dotenv.load_dotenv()


def get_mi_device():
    # 读取所有环境变量
    env_vars = os.environ

    # 过滤以 MI_DID 和 MI_MODEL 开头的环境变量
    mi_vars = {
        key: value
        for key, value in env_vars.items()
        if key.startswith("MI_DID") or key.startswith("MI_MODEL")
    }

    device_dict = defaultdict(list)  # 使用 defaultdict
    for key, value in mi_vars.items():
        # 获取设备名称
        number = int(key.replace("MI_DID", "").replace("MI_MODEL", "") or 0)
        device_dict[number].append(value)  # 直接添加到列表中

    device_list = list(device_dict.values())
    # 将结果按组输出
    return {item[0]: item[1] for item in device_list}


if __name__ == "__main__":
    get_env()

    print(get_mi_device())
