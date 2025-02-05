import subprocess
import sys
import nltk


def install_requirements():
    print("正在安装依赖...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", "requirements.txt"])

def install_browser():
    print("正在安装浏览器...")
    subprocess.check_call([sys.executable, "-m", "playwright", "install"])

def download_nltk_data():
    print("正在下载NLTK词典文件...")
    nltk.download('vader_lexicon')

def download_model():
    try:
        import ollama
        ollama.pull('qwen2.5')
    except Exception as e:
        print(f"下载模型失败：{e}")

def main():
    try:
        install_requirements()
        install_browser()
        download_nltk_data()
        download_model()
        print("所有安装步骤已完成！")
    except subprocess.CalledProcessError as e:
        print(f"安装过程中出现错误：{e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
