# encoding: utf-8
'''
将数据存储到数据库
'''

import sqlite3
import pandas as pd
from collections import Counter
import json

class NewsDatabase:
    def __init__(self, db_name='news.db'):
        self.db_name = db_name
        self.create_tables()

    def create_tables(self):
        conn = sqlite3.connect(self.db_name)
        cursor = conn.cursor()
        
        cursor.execute('''CREATE TABLE IF NOT EXISTS news
                          (id INTEGER PRIMARY KEY AUTOINCREMENT,
                           日期 TEXT,
                           时间 TEXT,
                           信息来源 TEXT,
                           标题 TEXT,
                           排名 INTEGER,
                           热度 TEXT,
                           处理后热度 TEXT,
                           链接 TEXT,
                           情感得分 REAL,
                           加权情感得分 REAL,
                           分词 TEXT)''')
        
        cursor.execute('''CREATE TABLE IF NOT EXISTS raw_html
                          (id INTEGER PRIMARY KEY AUTOINCREMENT,
                           日期 TEXT,
                           时间 TEXT,
                           内容 TEXT)''')
        
        conn.commit()
        conn.close()

    def save_data(self, df: pd.DataFrame, table_name='news'):
        conn = sqlite3.connect(self.db_name)
        
        if table_name == 'news':
            df['分词'] = df['分词'].apply(json.dumps)
        
        df.to_sql(table_name, conn, if_exists='append', index=False)
        
        conn.close()

    def save_raw_html(self, html_content: str):
        df = pd.DataFrame({
            '日期': [pd.Timestamp.now().strftime('%Y-%m-%d')],
            '时间': [pd.Timestamp.now().strftime('%H:%M:%S')],
            '内容': [html_content]
        })
        self.save_data(df, 'raw_html')

    def read_data(self, query="SELECT * FROM news"):
        conn = sqlite3.connect(self.db_name)
        
        df = pd.read_sql_query(query, conn)
        if '分词' in df.columns:
            df['分词'] = df['分词'].apply(json.loads)
        
        conn.close()
        return df

    def execute_query(self, query, params=None):
        conn = sqlite3.connect(self.db_name)
        cursor = conn.cursor()
        
        if params:
            cursor.execute(query, params)
        else:
            cursor.execute(query)
        
        conn.commit()
        conn.close()

    def fetch_data(self, query, params=None):
        conn = sqlite3.connect(self.db_name)
        cursor = conn.cursor()
        
        if params:
            cursor.execute(query, params)
        else:
            cursor.execute(query)
        
        rows = cursor.fetchall()
        conn.close()
        return rows

    def delete_table(self,table_name:str):
        """删除news子表"""
        conn = sqlite3.connect(self.db_name)
        try:
            with conn:
                conn.execute(f"DROP TABLE IF EXISTS {table_name}")
            print("news表已成功删除")
        except sqlite3.Error as e:
            print(f"删除news表时发生错误：{e}")

    def clear_table(self, table_name):
        """清空指定表的所有数据"""
        conn = sqlite3.connect(self.db_name)
        try:
            with conn:
                conn.execute(f"DELETE FROM {table_name}")
            print(f"{table_name}表已成功清空")
        except sqlite3.Error as e:
            print(f"清空{table_name}表时发生错误：{e}")

    def export_table_data(self, query="SELECT * FROM news WHERE 日期 = (SELECT MAX(日期) FROM news)"):
        """导出表数据"""
        conn = sqlite3.connect(self.db_name)
        df = pd.read_sql_query(query, conn)
        conn.close()
        data_time = str(df["日期"].iloc[0])
        # 将标题和链接合并成HTML链接格式
        df['标题'] = df.apply(lambda row: f'<a href="{row["链接"]}">{row["标题"]}</a>', axis=1)
        df = df[["信息来源","标题"]]  # 只保留信息来源和带链接的标题列
        return df

    def export_word_cloud_data(self,  query="SELECT 分词 FROM news WHERE 日期 = (SELECT MAX(日期) FROM news)"):
        """导出词云数据"""
        rows = self.fetch_data(query)
        words = []
        for row in rows:
            words.extend(eval(row[0]))  # 使用 eval 来解析字符串列表
        word_counts = Counter(words)
        word_cloud_data = [(word,count) for word, count in word_counts.items()]
        return word_cloud_data 

    def export_pie_chart_data(self,  query="SELECT 信息来源, SUM(CAST(处理后热度 AS INTEGER)) FROM news WHERE 日期 = (SELECT MAX(日期) FROM news) GROUP BY 信息来源"):
        """导出饼图数据"""
        rows = self.fetch_data(query)
        pie_chart_data = [(row[0],row[1]) for row in rows]
        return pie_chart_data

    def export_line_chart_data(self, query = """
        SELECT 日期, SUM(加权情感得分) as total_sentiment
        FROM news
        GROUP BY 日期
        ORDER BY 日期
    """):
        """导出折线图数据"""
        
        rows = self.fetch_data(query)
        line_chart_data = [(row[0],  int(row[1])) for row in rows]
        return line_chart_data

    def export_bar_chart_data(self,  query="""
        SELECT 信息来源, 加权情感得分
        FROM news
        WHERE 日期 = (SELECT MAX(日期) FROM news)
    """):
        """导出柱状图数据"""
        rows = self.fetch_data(query)
        bar_chart_data = {}
        for row in rows:
            source = row[0]
            sentiment_score = row[1]
            if source not in bar_chart_data:
                bar_chart_data[source] = {"positive": 0, "negative": 0}
            if sentiment_score > 0:
                bar_chart_data[source]["positive"] += sentiment_score
            elif sentiment_score < 0:
                bar_chart_data[source]["negative"] += sentiment_score
        
        formatted_data = [ (source, int(data["positive"]), abs(int(data["negative"])),abs(int(data["positive"])+int(data["negative"])))
                          for source, data in bar_chart_data.items()]
        return formatted_data

