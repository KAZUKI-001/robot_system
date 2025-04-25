import os
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import csv


def list_rosbag_directories(directory):
    """
    指定されたディレクトリ内のすべてのROS 2バッグディレクトリを取得。
    バッグディレクトリとは、.db3 と .yaml が含まれるディレクトリとする。
    """
    try:
        directories = []
        for subdir in os.listdir(directory):
            full_path = os.path.join(directory, subdir)
            if os.path.isdir(full_path):  # サブディレクトリかどうか確認
                # .db3 と .yaml が存在するか確認
                db3_exists = any(f.endswith('.db3') for f in os.listdir(full_path))
                yaml_exists = any(f.endswith('.yaml') for f in os.listdir(full_path))
                if db3_exists and yaml_exists:
                    directories.append(subdir)
        if not directories:
            print("指定されたディレクトリにROS 2バッグディレクトリが見つかりません。")
            return []
        return directories
    except FileNotFoundError:
        print(f"ディレクトリ '{directory}' が見つかりません。")
        return []

def select_rosbag_directory(directories):
    """
    ユーザーにバッグディレクトリのリストを提示して選択させる。
    """
    if not directories:
        return None

    print("以下のROS 2バッグディレクトリが見つかりました:")
    for i, dir_name in enumerate(directories):
        print(f"[{i + 1}] {dir_name}")

    while True:
        try:
            choice = int(input(f"変換するバッグディレクトリの番号を選んでください（1〜{len(directories)}）： "))
            if 1 <= choice <= len(directories):
                return directories[choice - 1]
            else:
                print("無効な番号です。もう一度選択してください。")
        except ValueError:
            print("無効な入力です。数字を入力してください。")

def main():
    # バッグディレクトリを検索するディレクトリを指定
    #directory = input("ROS 2バッグファイルが格納されているディレクトリを入力してください: ").strip()
    directory = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),'bags')

    # ディレクトリ内のバッグディレクトリを取得
    rosbag_dirs = list_rosbag_directories(directory)
    if not rosbag_dirs:
        return

    # ユーザーに選択させる
    selected_dir = select_rosbag_directory(rosbag_dirs)
    if selected_dir:
        # 絶対パスに変換
        bag_dir_path = os.path.abspath(os.path.join(directory, selected_dir))
        print(f"選択されたバッグディレクトリ: {bag_dir_path}")
		
        parse_bag(bag_dir_path)

        # ここにバッグファイルの変換処理を記述
        # 例: process_rosbag_directory(bag_dir_path)
        print(f"バッグディレクトリ '{bag_dir_path}' を処理中...")

def get_first_db3_file(directory):
    """
    指定されたディレクトリ内で最初に見つかった .db3 ファイルの絶対パスを返す
    """
    try:
        # ディレクトリ内のすべてのファイルをチェック
        for f in os.listdir(directory):
            if f.endswith('.db3'):
                # 最初に見つかった .db3 ファイルの絶対パスを返す
                return os.path.abspath(os.path.join(directory, f))
        # .db3 ファイルが見つからなかった場合
        print("指定されたディレクトリに .db3 ファイルが見つかりません。")
        return None
    except FileNotFoundError:
        print(f"ディレクトリ '{directory}' が見つかりません。")
        return None

def parse_bag(dir_path):
    # 入力バッグファイル
    BAG_PATH = dir_path
    # 対象トピック
    TOPIC_NAME = input('保存するトピック名を入力してください')
    # 出力先CSVファイル
    OUTPUT_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)),'outputs',TOPIC_NAME.lstrip('/')+'output.csv')

    # CSV 書き込みの準備
    with open(OUTPUT_CSV, mode='w', newline='') as csvfile:
        csv_writer = None

        # ROS 2 バッグを開く
        with Reader(BAG_PATH) as reader:
            # トピックとメッセージタイプの確認
            connections = [x for x in reader.connections if x.topic == TOPIC_NAME]
            if not connections:
                print(f"トピック名:{TOPIC_NAME} は存在しないトピック名です。")
                exit(1)

            # メッセージを反復処理
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = deserialize_cdr(rawdata, connection.msgtype)

                # 最初のメッセージでCSVヘッダーを設定
                if csv_writer is None:
                    csv_writer = csv.DictWriter(csvfile, fieldnames=vars(msg).keys())
                    csv_writer.writeheader()

                # データ行を書き込む
                csv_writer.writerow(vars(msg))

    print(f"{TOPIC_NAME}トピックは{OUTPUT_CSV}として保存しました.")

if __name__ == '__main__':
    main()

