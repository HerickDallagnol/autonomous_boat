# geometry_msgs
A Twist is message that contain 2 variables linear and angular of type geometry_msgs/Vector3. A geometry_msgs/Vector3 type contain 3 variables x, y, z of type float64. Simply a geometry_msgs/Twist is a data structure that can be used to write linear and angular speed. 

* **`twist_pub.py:`** : Initiates the 'Twist_node' module, facilitating the commencement of operations. This module facilitates the publication of the 'cmd/vel' topic, thereby affording users the capability to convey pertinent linear X and angular Z data.

* **`twist_sub.py:`** : An ESP-initiated node initializes by subscribing to the topic relayed by 'twist_pub,' enabling it to capture and process the data from the respective topic. This acquired information is then transmitted to the variables 'move1' and 'move2.' Practical validation of this process can be conducted by examining the voltage outputs detected at pins 25 and 26, corresponding to 'move1' and 'move2,' respectively.

