import rospy
import tf2_ros
import geometry_msgs.msg

def get_link_pose(tf_buffer, source_link, target_link):
    try:
        trans = tf_buffer.lookup_transform(target_link, source_link, rospy.Time(), rospy.Duration(1.0))
        return trans.transform.translation
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print("Error:", e)
        return None

if __name__ == "__main__":
    rospy.init_node('tf2_example')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    source_link = "base_link"  # ソースリンクをbase_linkに設定
    target_link = "gnss_link"  # ターゲットリンクをgnss_linkに設定

    rate = rospy.Rate(1)  # ループの頻度を1Hzに設定

    while not rospy.is_shutdown():
        link_pose = get_link_pose(tf_buffer, source_link, target_link)
        
        if link_pose:
            x_difference = link_pose.x
            y_difference = link_pose.y

            print(f"X Difference: {x_difference}, Y Difference: {y_difference}")

        rate.sleep()
