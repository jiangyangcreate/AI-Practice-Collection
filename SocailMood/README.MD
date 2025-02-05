# 代码说明

该项目适合作为小型的项目原型，适合教学和练手。最初这个项目的灵感源于我的个人需求，我需要一个工具来查看主流话题，同时又不想下载一堆 APP 来接收推送。

项目开源地址：[https://github.com/jiangyangcreate/AI-Practice-Collection/SocialMood](https://github.com/jiangyangcreate/AI-Practice-Collection/SocialMood)

## 项目目录结构

```bash showLineNumbers
SocialMood/
├── docs/              # 文档和静态页面目录
│   └── index.html     # 数据可视化的静态页面，展示词云图等图表
├── src/               # Python代码主目录
│   ├── _dataclean.py       # 数据清洗脚本，负责数据清洗和处理
│   ├── _model.py     # 模型脚本，负责情绪分析
│   ├── _pyechart.py    # 数据可视化脚本，负责数据可视化
│   ├── _database.py    # 数据库操作模块，处理SQLite数据存储
├── README.MD           # 项目说明文档，包含项目介绍和使用说明
├── .gitignore         # Git忽略文件配置，用于排除不需要版本控制的文件
├── .nojekyll         # 用于GitHub Pages的配置文件
├── requirements.txt # Python依赖包列表
├── setup.py # 项目依赖安装脚本，负责安装必要的包和模型
├── run.py # 项目运行脚本，负责运行爬虫和数据处理
├── news.db # SQLite数据库文件，存储新闻数据
```

## 使用流程

```bash
# 安装依赖
cd python
python setup.py

# 运行数据抓取，生成静态网页：html/charts.html
python crawler.py
```

### 依赖安装

所需依赖可以通过 `setup.py` 下载安装。因为有些模块不是pip就算安装好的。

### 主要功能

该系统的主要功能包括：
- **抓取热搜数据**：从微博、抖音、B站等平台抓取热搜数据。

这里也可以通过API获取，爬取注意不要变成DDOS攻击。

- **数据处理**：使用 Pandas 进行数据清洗与处理。

使用pandas主要是处理一些文本型的数据，譬如10万要换算为100000。

使用jieba分词用于后续词云图生成，需要剔除一些单字与标点符号。当然，最近b站很喜欢在标题中加空格，所以要先去空格再分词。

有些数据的热度值还没计算出来，可以使用幂律分布的线性回归填补热度缺失值。这里使用指数回归、普通线性回归效果都不好。


- **情绪分析**：监测和分析公众情绪。算出单条标题的情绪数值之后，标准化到 `(-1,1)` 这个区间之中。最后通过热度与排名计算出对社会的情感影响力。正数数则是积极影响，负数则是负面影响。

如果你的电脑性能还不错，可以使用本地模型作为情绪分析的核心，根据自己的设备选择模型大小。
### 数据存储

系统使用 `sqlite3` 保存中间数据，此部分为基本的增删改查，相关代码在 `_database.py` 中，数据文件为 `news.db`，包含两张表：
- **news**：清洗后的可用数据。
- **raw_html**：原始网页数据。

这样可以保证速度的同时，可以保持文件夹的整洁，如果数据量大可以直接平滑的迁移到正式数据库中。
### 数据可视化

接下来绘制一些图表，包括：
- 各个平台的情绪红绿图。
- 公众情绪的涨跌折线图。
- 每日全网词云图。

通过pyecharts的脚手架，导出为静态网页。这里为了代码直观，封装为了类。

为了保证协调统一，这里将所有图表的绘制都封装到了一个类中，都使用pyecharts。table的绘制使用pyecharts的components的Table类，这个类默认会将超链接转义，查看源代码发现内容有一个参数 escape_data 用于设置是否转义。但是没有被暴露出来，已经提交了[PR](https://github.com/pyecharts/pyecharts/pull/2416/files)，如果你的escape_data=False报错，可以自己修改源代码。

# 代码设置

随着时间的推移，爬虫部分的代码可能需要自己修改。你也可以在main函数中，将debug设为True，这样不会真的爬取，而是调用本地的已爬取的网页。生成后的内容在`docs/index.html`。
