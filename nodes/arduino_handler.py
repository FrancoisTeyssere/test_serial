#!/usr/bin/python
"""
Handles Arduino messages and publishes them to ROS topics
"""
import serial
import rospy
from std_msgs.msg import String, Bool
from roslib.message import get_message_class
from openag_brain.load_env_var_types import VariableInfo

start_button_pub = rospy.Publisher('/start_button', Bool, queue_size = 10)
top_switch_pub = rospy.Publisher('/top_switch', Bool, queue_size = 10)
bottom_switch_pub = rospy.Publisher('/bottom_switch', Bool, queue_size = 10)
torque_error_pub = rospy.Publisher('/torque_error', Bool, queue_size = 10)

def expand_unknown_status(status_code):
    return {
        "is_ok": False,
        "message": "Unknown status code {}".format(status_code)
    }

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

        # Zip values with the corresponding environmental variable
        return {"ok":pairs}
        #for header, value in pairs:
            #if VALID_SENSOR_VARIABLES[header]:


                #return {"ok":pairs}

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


    #actuator_csv_headers = rospy.get_param("~actuator_csv_headers", (
        #"status",
        #"drill",
        #"vacuum",
        #"light",
        #"motor_left",
        #"motor_right",
        #"motor_up"
    #))

    actuator_csv_headers = {
        "status":0,
        "drill":False,
        "vacuum":False,
        "light":0,
        "motor_left":0.0,
        "motor_right":0.0,
        "motor_up":0.0
        }


    #ENVIRONMENTAL_VARIABLES = frozenset(
       # VariableInfo.from_dict(d)
       # for d in rospy.get_param("/var_types/environment_variables").itervalues())

   # VALID_SENSOR_VARIABLES = [v for v in ENVIRONMENTAL_VARIABLES if v.name in sensor_csv_headers]

   # PUBLISHERS = {
     #   variable.name: rospy.Publisher(
   #         "{}/raw".format(variable.name),
  #          get_message_class(variable.type),
   #         queue_size=10)
    #    for variable in VALID_SENSOR_VARIABLES
   # }

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

    serial_port_id = rospy.get_param("~serial_port_id", "/dev/ttyACM0")
    publisher_rate_hz = rospy.get_param("~publisher_rate_hz", 1)
    baud_rate = rospy.get_param("~baud_rate", 115200)

    timeout_s = 1 / publisher_rate_hz
    # Initialize the serial connection
    serial_connection = serial.Serial(serial_port_id, baud_rate, timeout=timeout_s)
    prev_time = rospy.get_time()

    while not rospy.is_shutdown():

        # Read before writing
        buf = serial_connection.readline()

        # Generate the message for the current state
        # status, pump1, pump2, pump3, pump4, pump5, chiller_fan, chiller_pump, heater_core2, air_flush, water_aeration, water_circulation, chamber_fan, blue, white, red, heater_core1, chiller_compressor
        message = "0,{},{},{},{},{},{}\n".format(
            actuator_csv_headers["drill"],
            actuator_csv_headers["vacuum"],
            actuator_csv_headers["light"],
            actuator_csv_headers["motor_left"],
            actuator_csv_headers["motor_right"],
            actuator_csv_headers["motor_up"]
        ).encode('utf-8')
        serial_connection.write(message)
        serial_connection.flush()

        pairs_or_error = process_message(buf)
        if pairs_or_error.get("ok") is not None:
            pairs = pairs_or_error.get("ok")
        else:
            error_message = pairs_or_error
            ARDUINO_STATUS_PUBLISHER.publish(error_message)

        publish_time = ros_next(publisher_rate_hz, prev_time)

        if publish_time and pairs is not None:
            ARDUINO_STATUS_PUBLISHER.publish("OK")
            publish_topics(pairs)
            #for header, value in pairs:
                #PUBLISHERS[header].publish(value)
                #print(value)

                #continue


    serial_connection.close()
