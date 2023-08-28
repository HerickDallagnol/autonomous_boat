# autonomous_boat
Inside this repository, contains the packages for the autonomous boat project. Includes the GPS message package, the PWM message package and messages from the geometry_msgs library.

* **`geometry_msgs`** : This package publishes and receives a message of type geometry_msgs/ that contains variables x, z of type float64. Simply a geometry_msgs/Twist is a data structure that can be used to write linear and angular velocities.

*  **`pwm_control`** : This package creates a publisher (pub_pwm.py) to send a PWM signal value to the ESP, where a subscriber (pwm_subscriber.ino) updates the value in the ESP.

*  **`gps_node`** : Package with the code responsible for starting the gps node.
  
*  **`ConnectSSh_RaspPi4`** : Demonstration of how to connect to rasp via ssh





