#!/usr/bin/env python3
import rospy
import csv
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix

mcl_pose_data = []  # mcl_poseのデータを格納するリスト
gps_fix_data = []   # gps/fix/ichimillのデータを格納するリスト

def mcl_pose_callback(msg):
    global mcl_pose_data
    mcl_pose_data.append((rospy.Time.now(), msg))

def gps_fix_callback(msg):
    global gps_fix_data
    gps_fix_data.append((rospy.Time.now(), msg))

def main():
    rospy.init_node('your_node_name')  # ノードの初期化。your_node_nameはノードの名前を適切に設定する

    # mcl_pose トピックのサブスクライバーを作成
    rospy.Subscriber('mcl_pose', PoseWithCovarianceStamped, mcl_pose_callback)

    # gps/fix/ichimill トピックのサブスクライバーを作成
    rospy.Subscriber('gps/fix/ichimill', NavSatFix, gps_fix_callback)

    # CSVファイルのパス
    csv_file_path = '/path/to/output.csv'  # 適切なパスに変更する

    # ループを1Hzで実行する
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # ここに他の処理が必要な場合は追加する

        rate.sleep()

        # 1秒ごとにデータをCSVファイルに書き込む
        write_to_csv(csv_file_path)

def write_to_csv(file_path):
    global mcl_pose_data, gps_fix_data

    # mcl_poseデータをCSVファイルに書き込む
    with open(file_path, 'w') as csv_file:
        csv_writer = csv.writer(csv_file)

        # ヘッダーを書き込む
        csv_writer.writerow(['Timestamp', 'mcl_pose_data', 'gps_fix_data'])

        # データを書き込む
        for mcl_data, gps_data in zip(mcl_pose_data, gps_fix_data):
            timestamp_mcl, mcl_msg = mcl_data
            timestamp_gps, gps_msg = gps_data

            csv_writer.writerow([timestamp_mcl, str(mcl_msg), str(gps_msg)])

if __name__ == '__main__':
    main()
