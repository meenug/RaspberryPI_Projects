# Initiate the rosbag for capture data from any ROS topic, when the gpio pin state on Pi is changed #

##pi_gpio.py##

Controls the Raspberry Pi GPIO pin and publishes the message to image_record subscriber. 

Need to be superuser to access gpio libraries and run this program.

Requires RPi.GPIO library:

    $ sudo pip install RPi.GPIO
    
- The message is published only when the pin state is changed from high to low or vice-versa.
- The message is published as 0 or 1.
- Enter the pin number from command line to publish on topic gpio_pin_n, where n is the pin number entered.

###Run:###
    $ rosrun <pkg> pi_gpio.py \<input pin#\> gpio_pin_n:=\<subscriber topic\>

where n is the input pin# in gpio_pin_n 

eg. Here gpio_pin_7 is mapped to subscriber topic image_record

    $ rosrun pi_gpio pi_gpio.py 7 gpio_pin_7:=image_record


##camera_record.cpp

Subscriber to start recording and stop recording data when the message is 1(high) and stop when it's 0(low).

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;message type: std_msgs::String

      message value: "1" or "0"

                   1 - to start recording

                   0 - to stop recording

The bag file is created in the directory this program is run from.
###Run:###
    $ rosrun <pkg> camera_record
      
