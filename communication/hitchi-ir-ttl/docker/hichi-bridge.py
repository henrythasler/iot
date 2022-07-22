#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from time import sleep, time
from datetime import datetime
import paho.mqtt.client as mqtt
import json

import serial

SILENT = 0
ERROR = 1
INFO = 2
TRACE = 3


def millis(): return int(round(time() * 1000))


class EnergyMeter(object):
    def __init__(self, port="/dev/ttyUSB0", debug_level=SILENT, timeout=1):
        self.debug_level = debug_level
        self.connected = False

        self.serialInterface = serial.Serial(
            port=port,
            baudrate=300,
            bytesize=7,
            parity=serial.PARITY_EVEN,
            stopbits=1,
            timeout=timeout)

        if self.serialInterface.is_open:
            self.debug("opened port {}".format(port), INFO)
        else:
            self.debug("Could not open {}".format(port), ERROR)

    def __enter__(self):
        """Class can be used in with-statement"""
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.serialInterface.close()

    def debug(self, message, level=0):
        """Debug output depending on debug level."""
        if self.debug_level >= level:
            print(message)

    def readConsumption(self):
        if self.serialInterface.is_open:
            self.serialInterface.write(b"/?!\x0d\x0a")

            line = ""
            while not '1.8.0(' in str(line):
                line = self.serialInterface.readline()

            if '1.8.0(' in str(line):
                return float(line[6:14])
        else:
            self.debug("Serial interface not available", ERROR)
        return None


class Mqtt(object):
    def __init__(self, host="localhost", debug_level=SILENT):
        self.debug_level = debug_level
        self.host = host
        self.connected = False

        self.client = mqtt.Client('iot-{}-{}'.format("meter", os.getpid()))
        self.client.on_connect = self.on_connect

        self.client.connect(self.host)
        self.client.loop_start()

    def __enter__(self):
        """Class can be used in with-statement"""
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.client.loop_stop()
        self.client.disconnect()

    def debug(self, message, level=0):
        """Debug output depending on debug level."""
        if self.debug_level >= level:
            print(message)

    def publish(self, topic, data, retain=False):
        if self.connected:
            self.client.publish(topic, data, retain)

    def on_connect(self, client, userdata, flags, rc):
        self.debug("Connected to mqtt broker: " + self.host, TRACE)
        self.connected = True

    def publishObject(self, topic, value, unit, retain=False):
        dict = {
            "value": value,
            "timestamp": int(time()),
            "unit": unit
        }
        self.publish(topic, json.dumps(dict), retain)


if __name__ == "__main__":
    with Mqtt(host="omv4", debug_level=TRACE) as mqtt_client:
        with EnergyMeter(port="/dev/ttyUSB0", debug_level=TRACE) as meter:

            wattage = None
            prev_consumption = None
            prev_time = time()

            try:
                while True:
                    consumption = meter.readConsumption()

                    if consumption:
                        if prev_consumption and (consumption > prev_consumption):
                            wattage = (consumption - prev_consumption) * \
                                3600 / (time() - prev_time) * 1000
                            prev_time = time()
                            prev_consumption = consumption

                            mqtt_client.publishObject("home/energy/power/wattage", round(wattage, 2), "W", True)
                            mqtt_client.publish("home/energy/power/wattage/value", '{0:0.1f}'.format(wattage), True)

                            mqtt_client.publishObject("home/energy/power/consumption", consumption, "kWh", True)
                            mqtt_client.publish("home/energy/power/consumption/value", '{0:0.1f}'.format(consumption), True)

                        if not prev_consumption:
                            prev_consumption = consumption
                            prev_time = time()

                            mqtt_client.publishObject("home/energy/power/consumption", consumption, "kWh", True)
                            mqtt_client.publish("home/energy/power/consumption/value", '{0:0.1f}'.format(consumption), True)

                    sleep(60)

            except KeyboardInterrupt:
                print("cancel")
