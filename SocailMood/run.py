
from src._pyechart import ChartGenerator
from src._database import NewsDatabase
from src._dataclean import WebScraper, DataProcessor

def main(spider=False):
    db = NewsDatabase()
    if spider:
        scraper = WebScraper()
        html_content = scraper.fetch_webpage_content()
        db.save_raw_html(html_content)  # 保存原始HTML
    else:
        db.delete_table("news")
        db = NewsDatabase()
    html_contents = db.read_data("SELECT * FROM raw_html")

    for _, row in html_contents.iterrows():
        html_content = row["内容"]
        date = row["日期"]
        time = row["时间"]
        processor = DataProcessor(html_content, date, time)
        result_df = processor.run()
        db.save_data(result_df)
    
    # 调用绘图方法
    chart_gen = ChartGenerator()

    # 获取数据
    word_cloud_data = db.export_word_cloud_data()
    pie_chart_data = db.export_pie_chart_data()
    line_chart_data = db.export_line_chart_data()
    bar_chart_data = db.export_bar_chart_data()
    table_data = db.export_table_data()
    # 创建图表
    word_cloud = chart_gen.create_wordcloud(word_cloud_data)
    pie_chart = chart_gen.create_pie_chart(pie_chart_data)
    line_chart = chart_gen.create_line_chart(line_chart_data)
    bar_chart = chart_gen.create_bar_chart(bar_chart_data)
    table = chart_gen.create_table(table_data)
    # 渲染图表
    charts = [
        word_cloud, 
              pie_chart, 
              line_chart,
              bar_chart,
              table
              ]
    chart_gen.render_charts(charts)
if __name__ == "__main__":
    main()
