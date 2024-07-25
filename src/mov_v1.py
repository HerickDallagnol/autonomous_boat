#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float32, Int32

TOLERANCE_RADIUS = 7.0  # Tolerance radius for reaching the goal
FORWARD_VEL = 200    # Robot car forward velocity
SLOW_TURN_VEL = 200  # Slow turn velocity
PIVOT_WHEEL_VEL = 50  # Pivot wheel velocity
FAST_TURN_VEL = 150 
MAX_HEADING_ANGLE = 180
MIN_HEADING_ANGLE = 10
ANGLE_RANGE_DIV = 0.25
K_RIGHT_MOTOR = 1.0
K_LEFT_MOTOR = 1.0

# Mensagens PWM
pub_joy_y = rospy.Publisher('/vel_linear', Int32, queue_size=10)
pub_joy_x = rospy.Publisher('/vel_angular', Int32, queue_size=10)

# Waypoints
nav_waypoints = [
    {"lat": -31.78106117, "lon": -52.3233413},
    {"lat": -31.7810401, "lon": -52.323451},
    {"lat": -31.7810440, "lon": -52.32344818},
    {"lat": -31.781021118, "lon": -52.32344436}
]

waypoint_index = 0
num_waypoints = len(nav_waypoints)

# compass and gps
compass_angle = 0.0
waypoint_angle = 0.0
last_calc_dist = 0.0
gps_lat = 0.0
gps_lon = 0.0

#moving average filter
NUM_FILTERING_POINTS = 8
buffer_gps_lat = [0.0] * NUM_FILTERING_POINTS
buffer_gps_lon = [0.0] * NUM_FILTERING_POINTS
buffer_fill_index = 0

def store_gps_reading(lat, lon):
    global buffer_gps_lat, buffer_gps_lon, buffer_fill_index

    for i in range(NUM_FILTERING_POINTS - 1, 0, -1):
        buffer_gps_lat[i] = buffer_gps_lat[i - 1]
        buffer_gps_lon[i] = buffer_gps_lon[i - 1]

    #ponteiro
    buffer_gps_lat[0] = lat
    buffer_gps_lon[0] = lon
    
    if buffer_fill_index < NUM_FILTERING_POINTS:
        buffer_fill_index += 1

def compute_filtered_gps():
    if buffer_fill_index == 0:
        return {"lat": gps_lat, "lon": gps_lon}

    lat_sum = sum(buffer_gps_lat[:buffer_fill_index])
    lon_sum = sum(buffer_gps_lon[:buffer_fill_index])

    filtered_waypoint = {
        "lat": lat_sum / float(buffer_fill_index),
        "lon": lon_sum / float(buffer_fill_index)
    }
    return filtered_waypoint

# ROS gps
def gps_latitude_callback(data):
    global gps_lat
    gps_lat = data.data
    store_gps_reading(gps_lat, gps_lon)

def gps_longitude_callback(data):
    global gps_lon
    gps_lon = data.data
    store_gps_reading(gps_lat, gps_lon)

# ROS compass
def compass_callback(data):
    global compass_angle
    compass_angle = data.data  

def get_waypoint_with_index(index):
    return nav_waypoints[index]

def compute_navigation_vector(gps_lat, gps_lon):
    global waypoint_angle, last_calc_dist, waypoint_index

    cur_waypoint = get_waypoint_with_index(waypoint_index)

    delta_lat = math.radians(cur_waypoint["lat"] - gps_lat)
    gps_f_lat_rad = math.radians(gps_lat)
    waypoint_lat_rad = math.radians(cur_waypoint["lat"])
    delta_lon = math.radians(cur_waypoint["lon"] - gps_lon)

    a_haversine = math.sin(delta_lat / 2.0) ** 2 + math.cos(gps_f_lat_rad) * math.cos(waypoint_lat_rad) * math.sin(delta_lon / 2.0) ** 2
    c_haversine = 2 * math.atan2(math.sqrt(a_haversine), math.sqrt(1.0 - a_haversine))
    d_haversine = 6371000.0 * c_haversine

    last_calc_dist = d_haversine

    if d_haversine < TOLERANCE_RADIUS:       
        Joy_y = 130  #stop
        Joy_x = 130  
        pub_joy_y.publish(Joy_y)
        pub_joy_x.publish(Joy_x)
        rospy.sleep(3) 

        waypoint_index += 1
        if waypoint_index < num_waypoints:
            next_waypoint = get_waypoint_with_index(waypoint_index)
            rospy.loginfo(f"Próxima coordenada: Latitude {next_waypoint['lat']}, Longitude {next_waypoint['lon']}")

    if waypoint_index == num_waypoints:
        rospy.signal_shutdown("All waypoints reached")  #

    gps_f_lon_rad = math.radians(gps_lon)
    waypoint_lon_rad = math.radians(cur_waypoint["lon"])

    waypoint_angle = math.atan2(math.sin(waypoint_lon_rad - gps_f_lon_rad) * math.cos(waypoint_lat_rad),
                                math.cos(gps_f_lat_rad) * math.sin(waypoint_lat_rad) -
                                math.sin(gps_f_lat_rad) * math.cos(waypoint_lat_rad) * math.cos(waypoint_lon_rad - gps_f_lon_rad)) * 180 / math.pi

    if waypoint_angle < 0:
        waypoint_angle += 360

def control_navigation():
    global heading_error

    heading_error = waypoint_angle - compass_angle

    if heading_error < -180:
        heading_error += 360
    if heading_error > 180:
        heading_error -= 360

    if MIN_HEADING_ANGLE < heading_error <= MAX_HEADING_ANGLE * ANGLE_RANGE_DIV:
        # Right
        Joy_y = 130
        Joy_x = 20      
    elif MAX_HEADING_ANGLE * ANGLE_RANGE_DIV < heading_error <= MAX_HEADING_ANGLE:
        # Right fast
        Joy_y = 130
        Joy_x = 0 
    elif -MAX_HEADING_ANGLE <= heading_error < -MAX_HEADING_ANGLE * ANGLE_RANGE_DIV:
        # Left
        Joy_y = 130  
        Joy_x = 180  
    elif -MAX_HEADING_ANGLE * ANGLE_RANGE_DIV <= heading_error < -MIN_HEADING_ANGLE:
        # Left fast
        Joy_y = 130  
        Joy_x = 255 
    elif -MIN_HEADING_ANGLE <= heading_error <= MIN_HEADING_ANGLE:
        # Forward
        Joy_y = 200  
        Joy_x = 130  
    else:
        # Not defined
        Joy_y = 130  
        Joy_x = 130

    pub_joy_y.publish(Joy_y)
    pub_joy_x.publish(Joy_x)

    rospy.loginfo(f"Joy_y: {Joy_y}, Joy_x: {Joy_x}")

def initialize_joystick():
    Joy_y = 130  
    Joy_x = 130  
    pub_joy_y.publish(Joy_y)
    pub_joy_x.publish(Joy_x)
    rospy.sleep(5)

def main():
    rospy.init_node('boat_navigation', anonymous=True)
    rospy.Subscriber("compass_data", Float32, compass_callback)
    rospy.Subscriber("gps_latitude", Float32, gps_latitude_callback)
    rospy.Subscriber("gps_longitude", Float32, gps_longitude_callback)

    #função inicial da cadeira
    initialize_joystick()
   
    while not rospy.is_shutdown():
        gps_filtered = compute_filtered_gps()
        compute_navigation_vector(gps_filtered["lat"], gps_filtered["lon"])
        control_navigation()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
