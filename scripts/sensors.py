#! /usr/bin/env python

import rospy
from sense_hat import SenseHat
from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
    
    sense = SenseHat()
    sense.set_imu_config(True, True, True)

    freq = 100
    rospy.init_node("sense_hat", anonymous=True)
    rospy.loginfo(rospy.get_caller_id() + " Initializing node with frequency %s Hz", freq)
    rate = rospy.Rate(freq)

    pub_humidity = rospy.Publisher('sensehat/humidity', Float64, queue_size=10)
    pub_temperature = rospy.Publisher('sensehat/temperature', Float64, queue_size=10)
    pub_pressure = rospy.Publisher('sensehat/pressure', Float64, queue_size=10)
    pub_accelerometer = rospy.Publisher('sensehat/accelerometer', Vector3, queue_size=10)
    pub_gyroscope = rospy.Publisher('sensehat/gyroscope', Vector3, queue_size=10)
    pub_orientation_quat = rospy.Publisher('sensehat/orientation_quat', Quaternion, queue_size=10)
    pub_stick_dir = rospy.Publisher('sensehat/stick_dir', String, queue_size=10)
    pub_stick_state= rospy.Publisher('sensehat/stick_state', String, queue_size=10)


    try:

        rospy.loginfo(rospy.get_caller_id() + " Starting...")
        while not rospy.is_shutdown():

            acc_vec = Vector3()
            acceleration = sense.get_accelerometer_raw()
            acc_vec.x = acceleration['x']
            acc_vec.y = acceleration['y']
            acc_vec.z = acceleration['z']
            pub_accelerometer.publish(acc_vec)

            gyro_vec = Vector3()
            gyroscope  = sense.get_gyroscope()
            gyro_vec.x = gyroscope['roll']
            gyro_vec.y = gyroscope['pitch']
            gyro_vec.z = gyroscope['yaw']
            pub_gyroscope.publish(gyro_vec)

            ori_rad_pose = Quaternion()
            orientation_rad = sense.get_orientation_radians()
            quat = quaternion_from_euler(orientation_rad['roll'], orientation_rad['pitch'],orientation_rad['yaw'])
            ori_rad_pose.x = quat[0]
            ori_rad_pose.y = quat[1]
            ori_rad_pose.z = quat[2]
            ori_rad_pose.w = quat[3]
            pub_orientation_quat.publish(ori_rad_pose)

            pub_humidity.publish(sense.get_humidity())
            pub_temperature.publish(sense.get_temperature())
            pub_pressure.publish(sense.get_pressure())

            for event in sense.stick.get_events():
                if event.action == "pressed":
                    pub_stick_state.publish("Pressed")
                    pub_stick_dir.publish(event.direction)
                elif event.action == "released":
                    pub_stick_state.publish("Released")

            rate.sleep()

        rospy.loginfo(rospy.get_caller_id() + " Quit...")

    except rospy.ROSInterruptException:
        pass

