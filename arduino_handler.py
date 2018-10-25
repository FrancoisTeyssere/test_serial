#!/usr/bin/python
import serial
import random
import time

baud = 115200
# Open /dev/ttyACM0 with baudrate 9600, and defaults of 8N1
serial_connection = serial.Serial("/dev/ttyACM0", baud, timeout=1)

prev_time = time.time()
flush_period = 2000

if __name__ == "__main__":
    while True:

        # print("inWaiting: {}.".format(serial_connection.inWaiting()))
        now_time = time.time()
        # print(now_time - prev_time)
        prev_time = now_time
        try:
            buf = serial_connection.readline()

            drill = True
            vacuum = False
            light = False
            fwd_left = True
            fwd_right = False
            fwd_up = True
            rev_left = False
            rev_right = True
            rev_up = False
            stop_left = True
            stop_right = False
            stop_up = True
            speed_left = 0
            speed_right = 125
            speed_up = 254

            # status, pump1, pump2, pump3, pump4, pump5, chiller_fan, chiller_pump, heater_core2, air_flush, water_aeration, water_circulation, chamber_fan, blue, white, red, heater_core1, chiller_compressor
            message = "0,{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
                drill,
                vacuum,
                light,
                fwd_left,
                fwd_right,
                fwd_up,
                rev_left,
                rev_right,
                rev_up,
                stop_left,
                stop_right,
                stop_up,
                speed_left,
                speed_right,
                speed_up,
            ).encode('utf-8')

            serial_connection.write(message)
            serial_connection.flush()

            ary = buf.decode().split(',')
            if ary[0] != "0":
                print("status: {d[0]}, device: {d[1]}, code: {d[2]}".format(d=ary))
                continue
            # status, hum, temp, co2, water_temp, water_level_low, water_level_high, water_potential_hydrogen, water_electrical_conductivity
            print("status: {d[0]}, torque_error: {d[1]}, top_switch: {d[2]}, bottom_switch: {d[3]}, mid_switch: {d[4]}, start_button: {d[5]}, drill_button: {d[6]}, setting_button: {d[7]}, translator_up:{d[8]}, translator_down:{d[9]}, pot1:{d[10]}, pot2:{d[11]}".format(d=ary))
        # Short reads will show up as key errors since there are less per array
        except IndexError:
            print("Short read, received part of a message: {}".format(buf.decode()))
            serial_connection.close()
            serial_connection.open()
            continue
        # Occasionally, we get rotten bytes which couldn't decode
        except UnicodeDecodeError:
            print("Received weird bits, ignoring: {}".format(buf))
            serial_connection.close()
            serial_connection.open()
            continue
        # Stop elegantly on ctrl+c
        except KeyboardInterrupt:
            serial_connection.close()
            break
