#!/usr/bin/python3
import rospy
from sensor_msgs.msg import BatteryState

def BatteryStatePublisher():
    pub = rospy.Publisher('/battery_status', BatteryState, queue_size=10)
    rospy.init_node('BatteryStatePublisher', anonymous=True)
    rate = rospy.Rate(0.5) 

        
    # Initialize battery level
    battery_voltage = 9.0 # Initialize the battery voltage level
    percent_charge_level = 1.0  # Initialize the percentage charge level
    decrement_factor = 0.999 # Used to reduce battery level each cycle

    while not rospy.is_shutdown():
        msg = BatteryState() # Create a message of this type 
        msg.voltage = battery_voltage 
        msg.percentage = percent_charge_level

        battery_voltage = battery_voltage * decrement_factor
        percent_charge_level = percent_charge_level * decrement_factor
        
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        BatteryStatePublisher()
    except rospy.ROSInterruptException:
        pass