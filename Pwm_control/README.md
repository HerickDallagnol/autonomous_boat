# Pwm_nodos

Create a publisher (pub_pwm.py) to send a PWM signal value to the ESP, where a subscriber (pwm_subscriber.ino) updates the value on the ESP.

## Tutorial 
1 - Upload the code (pwm_subscriber.ino) to Esp32.

2 - Start roscore on the workspace

3 - In another terminal paste the (pub_pwm.py) in some pkg in your src

4 - run the (pub_pwm.py) in your terminal and put some value to pwm signal  

    $ python3 pub_pwm.py

![image](https://github.com/HerickDallagnol/Pwm_nodos/assets/140270456/af35ec8e-d17e-4cf8-b737-defa27b3e359)

4 - In another terminal start the node for the rossserial

    $ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=9600

  ![image](https://github.com/HerickDallagnol/Pwm_nodos/assets/140270456/5b5eec31-60f1-41ac-9840-276fc511af99)

5 - press En (Reset) button
