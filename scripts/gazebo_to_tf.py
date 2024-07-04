#!/usr/bin/env python3
import rospy
import tf
import math
from gazebo_msgs.msg import ModelStates #gazeboシミュレーションからモデルの状態を取得するためのメッセージ型をインポート

def callback(data): #gazebo/model_statesトピックからメッセージを受け取るコールバック関数を定義
    br = tf.TransformBroadcaster() #座標変換をブロードキャストするTFブロードキャスター作成
    try:
        index = data.name.index('kinect') #data.nameリストからkinectモデルのインデックス取得
        pose = data.pose[index] #data.poseリストからkinectモデルの姿勢情報取得

        # Gazeboからのクォータニオンを取得
        original_orientation = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )

        # 追加の座標変換を適用（x軸周りに-90度回転、z軸周りに-90度回転）
        additional_rotation = tf.transformations.quaternion_from_euler(-math.pi / 2, 0, -math.pi / 2)

        # クォータニオンの乗算を行い、最終的な回転を求める
        final_orientation = tf.transformations.quaternion_multiply(original_orientation, additional_rotation)

        # ブロードキャスト
        br.sendTransform((pose.position.x, pose.position.y, pose.position.z), #kinectモデルの位置
                         final_orientation, #最終的なクォータニオン
                         rospy.Time.now(), #現在のros時間
                         "camera_link", 
                         "world") #親フレームと子フレーム
    except ValueError: #kinectモデルがdata.nameにない場合何もしないでスルー
        pass

if __name__ == '__main__':
    rospy.init_node('gazebo_to_tf') #gazebo_to_tfという名前のrosノードを初期化
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback) #gazebo/model_statesトピックからメッセージを受け取るとcallback関数を呼び出す
    rospy.spin() #ROSノードが終了するまでループを維持

