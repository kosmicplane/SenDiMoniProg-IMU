#!/usr/bin/env python3
import rospy
import serial
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import tf

def imu_publisher():
    # Ajusta el puerto del ESP32 (ej: /dev/ttyUSB0 en Linux)
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

    rospy.init_node('imu_publisher', anonymous=True)
    pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)
    rate = rospy.Rate(50)  # frecuencia de publicación (50 Hz)

    imu_msg = Imu()

    while not rospy.is_shutdown():
        line = ser.readline().decode('utf-8').strip()
        if line:
            try:
                # Espera que el ESP32 mande 9 valores: ax,ay,az,gx,gy,gz,mx,my,mz
                ax, ay, az, gx, gy, gz, mx, my, mz = map(float, line.split(','))

                # Acelerómetro
                imu_msg.linear_acceleration.x = ax
                imu_msg.linear_acceleration.y = ay
                imu_msg.linear_acceleration.z = az

                # Giroscopio
                imu_msg.angular_velocity.x = gx
                imu_msg.angular_velocity.y = gy
                imu_msg.angular_velocity.z = gz

                # Orientación (ejemplo simplificado usando magnetómetro como referencia)
                quat = tf.transformations.quaternion_from_euler(mx, my, mz)
                imu_msg.orientation = Quaternion(*quat)

                # Publicar en el tópico
                pub.publish(imu_msg)

            except Exception as e:
                rospy.logwarn(f"Error parsing line: {e}")

        rate.sleep()

if __name__ == '__main__':
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass
