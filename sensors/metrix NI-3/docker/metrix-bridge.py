#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# docker run --rm -ti -p 5000:5000 registry:2
# docker build -t localhost:5000/python3-serial .
# docker push localhost:5000/python3-serial
# docker pull 192.168.178.30:5000/python3-serial
# docker run --rm -ti --device=/dev/ttyUSB1:/dev/ttyUSB1 -v /home/henry/dev:/host/dev 192.168.178.30:5000/python3-serial python /host/dev/metrix-bridge.py
# docker run -d --device=/dev/ttyUSB1:/dev/ttyUSB1 --name metrix_bridge -v /home/henry/dev:/host/dev 192.168.178.30:5000/python3-serial python /host/dev/metrix-bridge.py


import os
from time import sleep, time, mktime
from datetime import datetime, timezone
import paho.mqtt.client as mqtt
import json
import psycopg
import serial

SILENT = 0
ERROR = 1
INFO = 2
TRACE = 3


def millis(): return int(round(time() * 1000))

initialReading = None
meterReading = None
rawData = None
prev_time = 0

class GasMeter(object):
    def __init__(self, port="/dev/ttyUSB0", debug_level=SILENT, timeout=1):
        self.debug_level = debug_level
        self.connected = False
        self.port = port
        self.timeout = timeout

    def __enter__(self):
        """Class can be used in with-statement"""
        self.serialInterface = serial.Serial(
            port=self.port,
            baudrate=115200,
            timeout=self.timeout)

        self.debug("opened port {}".format(self.port), INFO)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.serialInterface.close()

    def debug(self, message, level=0):
        """Debug output depending on debug level."""
        if self.debug_level >= level:
            print(message)

    def readConsumption(self):
        if self.serialInterface.is_open:
            line = self.serialInterface.readline()
            try:
                data = json.loads(line)
            except:
                data = None
                if len(line):
                    print(line)
            return data
        else:
            self.debug("Serial interface not available", ERROR)
        return None


class Mqtt(object):
    def __init__(self, host="localhost", debug_level=SILENT):
        self.debug_level = debug_level
        self.host = host
        self.connected = False

    def __enter__(self):
        """Class can be used in with-statement"""
        self.client = mqtt.Client('iot-{}-{}'.format("gasmeter", os.getpid()))
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.client.connect(self.host)
        self.client.loop_start()

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
            self.client.publish(topic, data, qos=0, retain=retain)
        else:
            self.debug("No connection to MQTT broker!", ERROR)

    def on_connect(self, client, userdata, flags, rc):
        self.debug("Connected to mqtt broker: " + self.host, TRACE)
        self.connected = True
        client.subscribe("home/energy/gas/reference")

    def on_message(self, client, userdata, msg):
        global initialReading, rawData, prev_time
        
        self.debug("Received reference value: " + str(msg.topic) + ': ' + str(msg.payload), TRACE)
        
        prev_time = datetime.now(timezone.utc)
       
        if rawData:
            initialReading = float(msg.payload) - 0.01 * rawData["pulseCounter"]
            print("new meterReading: {} m³ on {}".format(initialReading + 0.01 * rawData["pulseCounter"], prev_time.strftime('%A %d-%m-%Y, %H:%M:%S')))
        else:
            initialReading = float(msg.payload)
            print("new meterReading: {} m³ on {}".format(initialReading, prev_time.strftime('%A %d-%m-%Y, %H:%M:%S')))


    def publishObject(self, topic, value, unit, retain=False):
        dict = {
            "value": value,
            "timestamp": int(time()),
            "unit": unit
        }
        self.publish(topic, json.dumps(dict), retain=retain)


if __name__ == "__main__":
    with Mqtt(host="omv4.fritz.box", debug_level=TRACE) as mqtt_client:
        with psycopg.connect("dbname='home' user='postgres' host='omv4.fritz.box' password='postgres'") as conn:
            with conn.cursor() as cur:
                with GasMeter(port="/dev/ttyUSB1", debug_level=TRACE, timeout=60) as meter:
                    
                    lastValues = cur.execute(
                        "SELECT * FROM consumption WHERE type='gas' ORDER BY timestamp DESC LIMIT 1").fetchone()
                    initialReading = lastValues[2]
                    prev_time = mktime(lastValues[0].timetuple())

                    print("Last database entry: {} m³ on {}".format(
                        initialReading, lastValues[0].astimezone(timezone.utc).strftime('%A %d-%m-%Y, %H:%M:%S')))

                    nextDbUpdate = int(time()) + 3600 * 6
                    meterReading = initialReading
                    
                    try:
                        while True:
                            rawData = meter.readConsumption()
                            if rawData:
                                meterReading = initialReading + 0.01 * rawData["pulseCounter"]
                                print("pulseCounter: {} => meterReading: {}".format(rawData["pulseCounter"], meterReading))
                                
                                mqtt_client.publishObject(
                                    "home/energy/gas/consumption", meterReading, "m³", retain=True)
                                mqtt_client.publish(
                                    "home/energy/gas/consumption/value", '{0:0.1f}'.format(meterReading), retain=True)

                                nextDbUpdate = int(time())
                                
                            if int(time()) >= nextDbUpdate:
                                cur.execute("INSERT INTO consumption (timestamp, type, value) VALUES (%s, 'gas', %s)", (
                                    datetime.now(timezone.utc), meterReading))
                                conn.commit()
                                nextDbUpdate = int(time()) + 3600 * 6

                            sleep(1)

                    except KeyboardInterrupt:
                        print("cancel")
