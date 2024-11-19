#!/usr/bin/env python3
import rospy
import tf
import math
from gazebo_msgs.msg import ModelStates

def callback(data):
    br = tf.TransformBroadcaster()
    try:
        for model_name in ['kinect1', 'kinect2', 'kinect3']:
            index = data.name.index(model_name)
            pose = data.pose[index]

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
                             f"{model_name}_camera_link", 
                             "map") #親フレームと子フレーム
    except ValueError: #kinectモデルがdata.nameにない場合何もしないでスルー
        pass

if __name__ == '__main__':
    rospy.init_node('gazebo_to_tf') #gazebo_to_tfという名前のrosノードを初期化
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback) #gazebo/model_statesトピックからメッセージを受け取るとcallback関数を呼び出す
    rospy.spin() #ROSノードが終了するまでループを維持
