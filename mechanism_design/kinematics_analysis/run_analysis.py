import os
from mechanism_analyzer import run_full_analysis


def main():
    # 这里指定您放在 data 文件夹下的 json 文件名
    json_filename = "Bennett.json"

    # 自动拼接路径：当前目录/data/文件名
    current_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(current_dir, 'data', json_filename)

    # 调用分析模块
    run_full_analysis(file_path)


if __name__ == "__main__":
    main()