"""
垃圾识别算法端
接收图片并保存，返回识别结果
"""
import dotenv
import os
dotenv.load_dotenv()
from typing import Dict, Any, Optional
import uuid
from langchain_community.document_loaders import Docx2txtLoader
from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_community.embeddings import DashScopeEmbeddings
from langchain_chroma import Chroma
from uuid import uuid4
from langchain_core.messages import HumanMessage, SystemMessage
from langgraph.graph import StateGraph, START, END
from typing import TypedDict
from langchain_openai import ChatOpenAI
from langchain_core.tools import tool
from langgraph.checkpoint.memory import MemorySaver
from langchain_tavily import TavilySearch
from openai import OpenAI
import base64
class Classifier:
    _New = None
    def __new__(cls,*args,**kwargs):
        if cls._New == None:
            cls._New = super().__new__(cls)
        return cls._New

    def __init__(self,path="测试数据.docx"):
        text_splitter = RecursiveCharacterTextSplitter(chunk_size=1000, chunk_overlap=100)
        loader = Docx2txtLoader(path)
        raw_documents = loader.load()

        documents = text_splitter.split_documents(raw_documents)
        embeddings = DashScopeEmbeddings(
            model="text-embedding-v3", dashscope_api_key=os.getenv("DASHSCOPE_API_KEY")
        )

        self.vector_store = Chroma(
            collection_name="example_collection",
            embedding_function=embeddings,
            persist_directory="./chroma_langchain_db",
        )
        # 为每个 chunk 生成 id
        uuids = [str(uuid4()) for _ in range(len(documents))]

        self.vector_store.add_documents(documents=documents, ids=uuids)

    def check_local(self,q="椰子壳是什么垃圾？"):
        return self.vector_store.similarity_search(
        q,
        k=3,
        )
    

@tool
def check_local_tool(q: str) -> str:
    """
    本地知识库查询

    参数:
        q: 查询关键词

    返回:
        str: 查询结果
    """
    results = Classifier().check_local(q)
    # 将文档列表转换为字符串
    return "\n\n".join([doc.page_content for doc in results])


def detect_image_format(image_base64: str) -> str:
    """
    通过检查 base64 数据的 magic bytes 自动检测图片格式
    
    参数:
        image_base64: base64编码的图片数据（可以是纯base64或包含data:image前缀的完整字符串）
    
    返回:
        str: 图片格式 (png, jpeg, webp, gif)，默认为 jpeg
    """
    try:
        # 如果包含 data:image 前缀，提取纯 base64 部分
        if "base64," in image_base64:
            image_base64 = image_base64.split("base64,")[1]
        
        # 解码 base64 获取二进制数据
        image_bytes = base64.b64decode(image_base64)
        
        # 检查 magic bytes 判断格式
        if image_bytes.startswith(b'\x89PNG\r\n\x1a\n'):
            return "png"
        elif image_bytes.startswith(b'\xff\xd8\xff'):
            return "jpeg"
        elif image_bytes.startswith(b'RIFF') and b'WEBP' in image_bytes[:12]:
            return "webp"
        elif image_bytes.startswith(b'GIF87a') or image_bytes.startswith(b'GIF89a'):
            return "gif"
        else:
            # 默认返回 jpeg（最常见的格式）
            return "jpeg"
    except Exception:
        # 如果检测失败，默认返回 jpeg
        return "jpeg"


@tool
def describe_image_tool(image_base64: str, image_format: Optional[str] = None):
    """
    获取图片描述
    
    参数:
        image_base64: 图片base64编码（可以是纯base64或包含data:image前缀的完整字符串）
        image_format: 图片格式 (png/jpeg/webp/gif)，如果不提供则自动检测
    
    返回:
        str: 图片描述文本
    """
    # 如果未提供格式，自动检测
    if image_format is None:
        image_format = detect_image_format(image_base64)
    
    # 如果 base64 字符串已经包含 data:image 前缀，直接使用
    if image_base64.startswith("data:image"):
        image_url = image_base64
    else:
        # 否则构建完整的 data URI
        image_url = f"data:image/{image_format};base64,{image_base64}"

    client = OpenAI(
        # 若没有配置环境变量，请用百炼API Key将下行替换为：api_key="sk-xxx"
        # 新加坡和北京地域的API Key不同。获取API Key：https://help.aliyun.com/zh/model-studio/get-api-key
        api_key=os.getenv('DASHSCOPE_API_KEY'),
        # 以下是北京地域base_url，如果使用新加坡地域的模型，需要将base_url替换为：https://dashscope-intl.aliyuncs.com/compatible-mode/v1
        base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
    )
    completion = client.chat.completions.create(
        model="qwen3-vl-plus", # 此处以qwen3-vl-plus为例，可按需更换模型名称。模型列表：https://help.aliyun.com/zh/model-studio/models
        messages=[
            {
                "role": "user",
                "content": [
                    {
                        "type": "image_url",
                        "image_url": {"url": image_url}, 
                    },
                    {"type": "text", "text": "描述图片中包含的物体"},
                ],
            }
        ],
    )
    return completion.choices[0].message.content

search_tool = TavilySearch(
            max_results=1,
            topic="general",
            # include_answer=False,
            # include_raw_content=False,
            # include_images=False,
            # include_image_descriptions=False,
            # search_depth="basic",
            # time_range="day",
            # include_domains=None,
            # exclude_domains=None,
            # country=None
            # include_favicon=False
            # include_usage=False
        )


llm = ChatOpenAI(
    api_key=os.getenv("DASHSCOPE_API_KEY"),
    base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
    model="qwen-plus",
)

checkpointer = MemorySaver()


# 定义 LangGraph 状态
class ImageClassificationState(TypedDict):
    """图片分类状态"""
    image_path: str  # 图片路径
    image_description: str  # 图片描述
    local_search_result: str  # 本地知识库检索结果
    online_search_result: str  # 在线检索结果
    final_answer: str  # 最终回答

# ==================== LangGraph 节点定义 ====================

def describe_image_node(state: ImageClassificationState) -> dict:
    """节点1: 获取图片描述"""
    image_path = state["image_path"]
    description = describe_image_tool.invoke({"image_base64": image_path})
    return {"image_description": description}


def search_local_node(state: ImageClassificationState) -> dict:
    """节点2: 根据图片描述进行本地知识库检索"""
    description = state["image_description"]
    result = check_local_tool.invoke({"q": f"搜索这个物品可能的垃圾分类结果：{description}"})
    return {"local_search_result": result}


def search_online_node(state: ImageClassificationState) -> dict:
    """节点3: 根据图片描述进行在线检索"""
    description = state["image_description"]
    result = search_tool.invoke({"query": description})
    return {"online_search_result": result}


def generate_answer_node(state: ImageClassificationState) -> dict:
    """节点4: 根据检索结果生成最终回答"""
    description = state.get("image_description", "")
    local_result = state.get("local_search_result", "")
    online_result = state.get("online_search_result", "")
    
    # 检查是否所有必要信息都已准备好
    if not description:
        return {"final_answer": "错误：图片描述未生成"}
    
    # 构建提示词
    prompt = f"""你是一个垃圾分类助手。请根据以下信息给出垃圾分类建议：

图片描述：{description}

在线检索结果：
{online_result if online_result else "未找到相关结果"}

本地知识库检索结果：
{local_result if local_result else "未找到相关结果"}

请综合分析以上信息，如果本地知识库和在线检索结果不一致，以本地知识库为准。

不要对本地知识库进行任何建议,如果本地知识库出现的内容与当前的垃圾分类的物品无关，请忽略。。

分类仅给出最终的垃圾分类建议和与之有关的信息。

分类结束后，为用户提供关于此垃圾的科普知识建议。
"""
    
    # 调用大模型生成回答
    response = llm.invoke([
        SystemMessage(content="你是一个专业的垃圾分类助手，能够根据图片描述和检索结果给出准确的分类建议。"),
        HumanMessage(content=prompt)
    ])
    
    return {"final_answer": response.content}


# ==================== 构建 LangGraph ====================

def build_classification_graph():
    """构建图片分类的 LangGraph"""
    # 创建图
    workflow = StateGraph(ImageClassificationState)
    
    # 添加节点
    workflow.add_node("describe_image", describe_image_node)
    workflow.add_node("search_local", search_local_node)
    workflow.add_node("search_online", search_online_node)
    workflow.add_node("generate_answer", generate_answer_node)
    
    # 添加边
    workflow.add_edge(START, "describe_image")
    workflow.add_edge("describe_image", "search_local")
    workflow.add_edge("describe_image", "search_online")
    # 等待两个搜索节点都完成后，再执行生成回答节点
    workflow.add_edge("search_local", "generate_answer")
    workflow.add_edge("search_online", "generate_answer")
    workflow.add_edge("generate_answer", END)
    
    # 编译图
    return workflow.compile(checkpointer=checkpointer)


# 创建全局图实例
classification_graph = build_classification_graph()


async def classify_image(image_base64: str) -> Dict[str, Any]:
    """
    分类图片的主函数（使用 LangGraph）
    
    参数:
        image_base64: base64编码的图片数据
    
    返回:
        dict: 包含分类结果的字典
            - success: bool
            - result: str (格式化的文本结果)
            - image_path: str (保存的图片路径)
    """
    try:

        # 使用 LangGraph 进行分类
        config = {"configurable": {"thread_id": str(uuid.uuid4())}}
        initial_state = {
            "image_path": image_base64,
            "image_description": "",
            "local_search_result": "",
            "online_search_result": "",
            "final_answer": ""
        }
        
        # 执行图
        result = classification_graph.invoke(initial_state, config)
        
        return {
            "success": True,
            "result": result["final_answer"],
            "image_path": image_base64,
            "image_description": result["image_description"],
            "local_search_result": result["local_search_result"],
            "online_search_result": result["online_search_result"]
        }
    except Exception as e:
        return {
            "success": False,
            "result": f"处理失败: {str(e)}",
            "image_path": None
        }

if __name__ == "__main__":
    def encode_image(image_path):
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode("utf-8")

    # 将xxxx/eagle.png替换为你本地图像的绝对路径

    image_base64= encode_image("test.webp")

    initial_state = {
                "image_path": image_base64,
                "image_description": "",
                "local_search_result": "",
                "online_search_result": "",
                "final_answer": ""
            }
    result = classification_graph.invoke(initial_state,  {"configurable": {"thread_id": str(uuid.uuid4())}})
    print(result )
