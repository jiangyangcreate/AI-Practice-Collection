from langchain_ollama import ChatOllama
from langchain.tools import tool
from langgraph.graph import StateGraph, START, END
import aiohttp
from pypetkitapi.client import PetKitClient
from pypetkitapi.command import FeederCommand
from langchain_core.messages import HumanMessage
import os
import asyncio
from typing_extensions import TypedDict
from typing import Annotated
from langgraph.graph.message import add_messages
from langchain_core.messages import trim_messages
from langchain_core.messages import ToolMessage
from langgraph.prebuilt import ToolNode, tools_condition
from dotenv import load_dotenv
load_dotenv()
# 设置语言模型
llm = ChatOllama(model="qwen2.5:14b")
class State(TypedDict):
    messages: Annotated[list, add_messages]
# 设置工具
@tool
def feed_pet(amount, feed_port = "amount2"):
    """
    Feed your pet (pet name is caicai 踩踩)
    amount: Amount of food to dispense (must be an integer)
    feed_port: Feeding port to use (amount1, amount2), defaults to amount2
    """
    # 验证出粮数量
    amount = max(0, min(int(amount), 2))
    # 验证出粮口参数
    if feed_port not in ['amount1', 'amount2']:
        return "错误：出粮口参数无效，必须是 'amount1' 或 'amount2'"
    
    async def _feed_pet():
        async with aiohttp.ClientSession() as session:
            client = PetKitClient(
                username=str(os.getenv("PETKIT_NAME")),
                password=str(os.getenv("PETKIT_PASSWORD")),
                region="CN",
                timezone="Asia/Shanghai",
                session=session,
            )
            await client.get_devices_data()
            feed_params = {feed_port: amount}
            await client.send_api_request(326799, FeederCommand.MANUAL_FEED, feed_params)
            return True

    try:
        asyncio.run(_feed_pet())
        return f"成功：已从端口{feed_port[-1]}投喂{amount}份食物。"
    except Exception as e:
        return f"错误：{str(e)}"

def select_next_node(state: State):
    # Otherwise, we can route as before
    return tools_condition(state)
tools = [feed_pet]
llm_with_tools = llm.bind_tools(tools)

def chatbot(state: State):
    
    trimmer = trim_messages(
        state["messages"],
        strategy="last",
        token_counter=len,
        max_tokens=10,
        start_on="human",
        end_on=("human", "tool"),
        include_system=True,
    )
    response = llm_with_tools.invoke(trimmer)
    return {"messages": [response]}

graph_builder = StateGraph(State)
graph_builder.add_node("chatbot", chatbot)
graph_builder.add_node("tools", ToolNode(tools=tools))
graph_builder.add_edge(START, "chatbot")
# 用于创建依赖于特定条件的连接。
graph_builder.add_conditional_edges(
    "chatbot",
    tools_condition,  # tools_condition,  # 用于创建依赖于特定条件的连接。
    {"tools": "tools", END: END},
)
# 用于创建固定的、无条件的连接。
graph_builder.add_edge("tools", "chatbot")
graph_builder.add_edge("tools", END)
graph = graph_builder.compile()

import random
def chat(prompt):
    results = graph.invoke(
        {"messages": [HumanMessage(content=prompt)]},
        config={"configurable": {"thread_id": str(random.randint(1, 1000000))}},
    )
    message= results["messages"][-1].content
    # 提取 tool_calls
    tool_calls = []
    for messages in results['messages']:
        if isinstance(messages, ToolMessage):
            tool_calls.append(messages.content)
    return message,tool_calls

if __name__ == "__main__":
    print(chat("出1份粮"))
