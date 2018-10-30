#!/usr/bin/python
"""
Handles Arduino messages and publishes them to ROS topics
"""
import serial
import math
import rospy
from std_msgs.msg import String, Bool, Int8, Float64
from geometry_msgs.msg import Twist
from roslib.message import get_message_class
from openag_brain.load_env_var_types import VariableInfo

start_button_pub = rospy.Publisher('/start_button', Bool, queue_size = 10)
top_switch_pub = rospy.Publisher('/top_switch', Bool, queue_size = 10)
bottom_switch_pub = rospy.Publisher('/bottom_switch', Bool, queue_size = 10)
torque_error_pub = rospy.Publisher('/torque_error', Bool, queue_size = 10)

last_drill_time = 0
last_light_time = 0
last_motors_time = 0
last_translator_time = 0

actuators_cmd = {
    "status":0,
    "drill":0,
    "vacuum":0,
    "light":0,
    "motor_left":0.0,
    "motor_right":0.0,
    "motor_up":0.0
    }


def cmd_callback(msg):


    global last_motors_time
    last_motors_time = rospy.get_time()
    WHEEL_RADIUS = 7.5
    ROBOT_LENGTH = 43.3
    GEAR_RATIO_R = 100
    GEAR_RATIO_L = 100

    #//Originally coded by Joel. Change variables names to make sense
    a = WHEEL_RADIUS / 2.0
    b = WHEEL_RADIUS / 2.0
    c = WHEEL_RADIUS / (2.0 * ROBOT_LENGTH)
    d = -WHEEL_RADIUS / (2.0 * ROBOT_LENGTH)
    e = msg.linear.x
    f = msg.angular.z
    determinant = a * d - b * c

    if (determinant != 0):

        desiredLeftMotorSpeed = (e * d - b * f) / determinant
        desiredRightMotorSpeed = (a * f - e * c) / determinant
        actuators_cmd["motor_left"] = desiredLeftMotorSpeed * GEAR_RATIO_L * 60 / (2 * math.pi)
        actuators_cmd["motor_left"] = desiredLeftMotorSpeed * GEAR_RATIO_L * 60 / (2 * math.pi)
        actuators_cmd["motor_right"] = -desiredRightMotorSpeed * GEAR_RATIO_R * 60 / (2 * math.pi)

def drill_callback(msg):
    global last_drill_time
    last_drill_time = rospy.get_time()
    actuators_cmd["drill"] = 1 if msg.data else 0
    actuators_cmd["vacuum"] = 1 if msg.data else 0

def light_cb(msg):
    global last_light_time
    last_light_time = rospy.get_time()
    actuators_cmd["light"] = msg.data

def translator_cb(msg):
    global last_translator_time
    last_translator_time = rospy.get_time()
    actuators_cmd["motor_up"] = -1*msg.data

def expand_unknown_status(status_code):
    return {
        "is_ok": False,
        "message": "Unknown status code {}".format(status_code)
    }

def check_time():
    curr_time = rospy.get_time()
    if curr_time - last_motors_time > 2:
        rospy.logerr("Motors timeout")
        actuators_cmd["motor_left"] = 0.0
        actuators_cmd["motor_right"] = 0.0

    if curr_time - last_translator_time > 2:
        rospy.logerr("Translator timeout")

        actuators_cmd["motor_up"] = 0.0

    if curr_time - last_light_time > 2:
        rospy.logerr("Light timeout")

        actuators_cmd["light"] = 0

    if curr_time - last_drill_time > 2:
        rospy.logerr("Drill timeout")

        actuators_cmd["drill"] = 0
        actuators_cmd["vacuum"] = 0


def ros_next(rate_hz, prev_time):
    timeout = 1 / rate_hz
    def closure(previous_time):
        curr_time = rospy.get_time()
        if curr_time - previous_time > timeout:
            previous_time = curr_time
            return True
        else:
            return False

    return closure(prev_time)

def publish_topics(pairs):
    start_button_pub.publish(int(pairs.get("start button"))==1)
    top_switch_pub.publish(int(pairs.get("top switch"))==1 or int(pairs.get("mid switch"))==1)
    bottom_switch_pub.publish(int(pairs.get("bottom switch"))==1)
    torque_error_pub.publish(int(pairs.get("torque error"))==1)

# Read the serial message string, and publish to the correct topics
def process_message(line):
    try:
        values = line.decode().split(',')
        print("SIZE : "+str(len(values)))
        #pairs = {k: v for k, v in zip(sensor_csv_headers, values)}
        pairs = dict(zip(sensor_csv_headers, values))
        #pairs = zip(sensor_csv_headers, values)
        status_code = pairs["status"]
        # Expand status code to status dict
        status = (
            STATUS_CODE_INDEX.get(status_code) or
            expand_unknown_status(status_code)
        )
        print pairs

        # WARN/ERR format: "status_code, device_name, message"
        if not status["is_ok"]:
            error_device = values[1]
            error_message = values[2]
            message = "{}> {}: {}".format(
                status["message"],
                error_device,
                error_message)
            rospy.logerr(message)
            return {"message":message}
        # status: OK

        return {"ok":pairs}


    except IndexError:
        rospy.logwarn("Short read, received part of a message: {}".format(buf.decode()))
        serial_connection.close()
        serial_connection.open()
        return {"nope":None}
    # Occasionally, we get rotten bytes which couldn't decode
    except UnicodeDecodeError:
        rospy.logwarn("Received weird bits, ignoring: {}".format(buf))
        serial_connection.close()
        serial_connection.open()
        return {"nope":None}

if __name__ == '__main__':
    rospy.init_node('handle_arduino')
    rospy.Subscriber("cmd_vel", Twist, cmd_callback)
    rospy.Subscriber("drill", Bool, drill_callback)
    rospy.Subscriber("light", Int8, light_cb)
    rospy.Subscriber("translator", Float64, translator_cb)


    sensor_csv_headers = ("status",
    "torque error",
    "top switch",
    "bottom switch",
    "mid switch",
    "start button",
    "drill button",
    "setting button",
    "translator up",
    "translator down",
    "pot1",
    "pot2")



    ARDUINO_STATUS_PUBLISHER = rospy.Publisher(
        "/arduino_status",
        String,
        queue_size=10)

    STATUS_CODE_INDEX = {
        "0": {
            "is_ok": True,
            "message": "OK"
        },
        "1": {
            "is_ok": False,
            "message": "WARN"
        },
        "2": {
            "is_ok": False,
            "message": "ERROR"
        }
    }

    serial_port_id = rospy.get_param("~serial_port_id", "/dev/arduino0")
    publisher_rate_hz = rospy.get_param("~publisher_rate_hz", 1)
    baud_rate = rospy.get_param("~baud_rate", 115200)

    timeout_s = 1 / publisher_rate_hz
    print("Publisher rate hz : "+str(publisher_rate_hz))
    print("baud rate : "+str(baud_rate))
    # Initialize the serial connection
    serial_connection = serial.Serial(serial_port_id, baud_rate, timeout=timeout_s, write_timeout=2)
    prev_time = rospy.get_time()
    last_input = 0
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():

        # Read before writing
        #serial_connection.flush()
        buf = ""
        try:
            buf = serial_connection.readline()

            print("Buffer : "+str(buf))

        except Exception as e:
            print(e)


        check_time() # check last actualisation of actuators

        # Generate the message for the current state
        # status, pump1, pump2, pump3, pump4, pump5, chiller_fan, chiller_pump, heater_core2, air_flush, water_aeration, water_circulation, chamber_fan, blue, white, red, heater_core1, chiller_compressor
        message = "0,{},{},{},{},{},{}\n".format(
            actuators_cmd["drill"],
            actuators_cmd["vacuum"],
            actuators_cmd["light"],
            actuators_cmd["motor_left"],
            actuators_cmd["motor_right"],
            actuators_cmd["motor_up"]
        ).encode('utf-8')
        print("Message sent : "+message)
        try:
            nb_bytes = serial_connection.write(message)
            print("nb_bytes : "+str(nb_bytes))
            #if(nb_bytes != 20):
                #print(nb_bytes)
               # print("Error : message not sent or incomplete")
        except:
            print("Write Timeout")

        #serial_connection.flushOutput()
        pairs = None
        pairs_or_error = {"ok":None}
        pairs_or_error = process_message(buf)

        if pairs_or_error.get("ok") is not None:
            pairs = pairs_or_error.get("ok")
            last_input = rospy.get_time()
        else:
            print("error, pairs not returned")
            error_message = pairs_or_error.get("message")
            ARDUINO_STATUS_PUBLISHER.publish(error_message)

        publish_time = ros_next(publisher_rate_hz, prev_time)

        if publish_time and pairs is not None:
            ARDUINO_STATUS_PUBLISHER.publish("OK")
            publish_topics(pairs)

        if rospy.get_time() - last_input > 5:
            ARDUINO_STATUS_PUBLISHER.publish("LOST_SYNC")


        rate.sleep()

    print("Closing serial connection...")
    serial_connection.close()
    print("Done")
