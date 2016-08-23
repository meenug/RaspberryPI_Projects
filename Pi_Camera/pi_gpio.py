#!/usr/bin/env python
###############################
# Reads the Raspberry Pi GPIO pin
# and publishes the message to image_record subscriber
# Need to be superuser to run this program and 
# access gpio libraries
# Requires: 
#     sudo pip install RPi.GPIO
# Usage: rosrun <pkg name> pi_gpio.py <input pin#>
#        rosrun <pkg name> pi_gpio.py <input pin#> gpio_pin_n:=image_record
#            where n is the input pin# in gpio_pin_n and 
#                  image_record is the subscriber topic
#
############################## 

import sys
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import String

# use P1 header pin numbering convention
GPIO.setmode(GPIO.BOARD)

def gpio_status(gpio_pin):
       
       pub = rospy.Publisher('gpio_pin_' + gpio_pin, String, queue_size=10, latch=True)
       rospy.init_node('pi_gpio_publisher', anonymous=True)

       HIGH = 1
       LOW = 0
       COUNT_NEEDED = 4
       times_high = 0
       times_low = 0
       pin_state = 0;
       current_state = LOW
   
       rate = rospy.Rate(40) # 10hz
       while not rospy.is_shutdown():

       	   # Input from pin 11
       	   pin_state = GPIO.input(int(gpio_pin))

	   if pin_state: # HIGH
		times_low = 0
            	times_high += 1
            	if times_high >= COUNT_NEEDED and current_state == LOW:
		 	current_state = HIGH
               		times_high = 0
               		rospy.loginfo("High")
               		pub.publish("1")

	   else: # LOW
		times_high = 0
        	times_low += 1
            	if times_low >= COUNT_NEEDED and current_state == HIGH:
		 	current_state = LOW
               		times_low = 0
               		rospy.loginfo("Low")
               		pub.publish("0")


           rate.sleep()


if __name__ == '__main__':
    try:
        numOfArg = len(sys.argv) 
        #print numOfArg
	if numOfArg <= 1:
		print "Usage:rosrun <pkg name> <pi_gpio.py> <input pin#>"
		sys.exit()

        pin = sys.argv[1]
        GPIO.setup(int(pin), GPIO.IN)  
        gpio_status(pin)

    except rospy.ROSInterruptException:
        pass

GPIO.cleanup()

