from pyecharts.charts import Bar, Line, Pie, WordCloud, Page
from pyecharts.components import Table
from pyecharts import options as opts
import os 
class ChartGenerator:
    def create_table(self,data) -> Table:
        """Generate a table."""
        table = Table()
        headers = data.columns.tolist()
        rows = data.values.tolist()
        table.add(headers, rows,escape_data=False)
        return table

    def create_bar_chart(self, data , y_label="Sentiment",title="正负面情绪") -> Bar:
        """Generate a bar chart with positive, negative, and absolute sentiment scores."""

        x_data, y_data_positive, y_data_negative, y_data_absolute = zip(*data)
        bar = (
            Bar()
            .add_xaxis(x_data)
            .add_yaxis("积极", y_data_positive, stack="stack1", category_gap="50%")
            .add_yaxis("消极", y_data_negative, stack="stack2", category_gap="50%")
            .add_yaxis("绝对值", y_data_absolute, stack="stack3", category_gap="50%")
            .set_global_opts(
                title_opts=opts.TitleOpts(title=title, pos_left="center"),
                xaxis_opts=opts.AxisOpts(name="来源"),
                yaxis_opts=opts.AxisOpts(name="情绪得分"),
                legend_opts=opts.LegendOpts(pos_top="10%", pos_right="10%"),
            )
        )
        return bar

    def create_line_chart(self,data,y_label="情绪得分",title="绝对情绪变化折线图") -> Line:
        """Generate a line chart."""
        x_data, y_data = zip(*data) 
        line = (
            Line()
            .add_xaxis(x_data)
            .add_yaxis(y_label, y_data, is_smooth=True)
            .set_global_opts(
                title_opts=opts.TitleOpts(title=title, pos_left="center"),
                xaxis_opts=opts.AxisOpts(name="日期"),
                legend_opts=opts.LegendOpts(is_show=False)
            )
        )
        return line

    def create_pie_chart(self, data_pairs, title="信息来源占比") -> Pie:
        """Generate a pie chart."""
        pie = (
            Pie()
            .add("", data_pairs)
            .set_global_opts(
                title_opts=opts.TitleOpts(title=title, pos_left="center"),
                legend_opts=opts.LegendOpts(pos_bottom="0%")  # Move legend to the bottom
            )
            .set_series_opts(label_opts=opts.LabelOpts(formatter="{b}: {d}%"))
        )
        return pie

    def create_wordcloud(self, words, title="词云") -> WordCloud:
        """Generate a word cloud."""
        wordcloud = (
            WordCloud()
            .add("", words, word_size_range=[20, 100],shape="circle")
            .set_global_opts(title_opts=opts.TitleOpts(title=title, pos_left="center"))
        )
        return wordcloud

    def render_charts(self, charts, output_file=None):
        """Render multiple charts to a single HTML file."""
        if output_file is None:
            output_file = os.path.join("docs", "index.html")

        page = Page(layout=Page.SimplePageLayout)
        page.add(*charts)
        page.render(output_file)
