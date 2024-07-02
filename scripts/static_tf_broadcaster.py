#!/usr/bin/env python3
import rospy
import tf
import math

if __name__ == '__main__':
    rospy.init_node('static_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        # y軸周りに-90度回転（z軸をx軸に合わせる）
        # z軸周りに180度回転（上下を反転させる）
        quaternion = tf.transformations.quaternion_from_euler(-math.pi / 2, 0, math.pi)
        br.sendTransform((0.0, 0.0, 0.0),
                         quaternion,
                         rospy.Time.now(),
                         "camera_link",
                         "world")
        rate.sleep()

