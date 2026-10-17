#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# docker run --rm -ti -p 5000:5000 registry:2
# docker build -t localhost:5000/python3-serial .
# docker push localhost:5000/python3-serial
# docker pull 192.168.178.30:5000/python3-serial
# docker run -d --device=/dev/ttyUSB0:/dev/ttyUSB0 -v /home/henry/dev:/host/dev 192.168.178.30:5000/python3-serial python /host/dev/hichi-bridge.py

import os
from time import sleep, time, mktime
from datetime import datetime, timezone
import paho.mqtt.client as mqtt
import json
import psycopg
import serial
from prometheus_client import start_http_server, Gauge
from smllib import SmlStreamReader

SILENT = 0
ERROR = 1
INFO = 2
TRACE = 3


def millis(): return int(round(time() * 1000))


class EnergyMeter(object):
    def __init__(self, port="/dev/ttyUSB0", debug_level=SILENT, timeout=1):
        self.serialInterface = None
        self.debug_level = debug_level
        self.connected = False
        self.port = port
        self.timeout = timeout
        self.reader = SmlStreamReader()
        self.obis_whitelist = {"1.8.0", "2.8.0"}    # Draw, Feed

    def __enter__(self):
        """Class can be used in with-statement"""
        self.serialInterface = serial.Serial(
            port=self.port,
            baudrate=9600,
            bytesize=8,
            parity=serial.PARITY_NONE,
            stopbits=1,
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
            result = {}
            data = self.serialInterface.read(4096)
            if not data:
                return None

            self.reader.add(data)
            frame = self.reader.get_frame()
            if not frame:
                return None

            # append required obis items to result list
            for entry in frame.get_obis():
                obis = entry.obis.obis_short
                if obis in self.obis_whitelist:
                    result[obis] = entry.get_value()

            return result
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
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, 'iot-{}-{}'.format("meter", os.getpid()))
        self.client.on_connect = self.on_connect

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

    def on_connect(self, client, userdata, flags, reason_code, properties):
        self.debug("Connected to mqtt broker: " + self.host, INFO)
        self.connected = True

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
                with EnergyMeter(port="/dev/ttyUSB0", debug_level=INFO) as meter:

                    lastValues = cur.execute(
                        "SELECT * FROM consumption WHERE type='strom' ORDER BY timestamp DESC LIMIT 1").fetchone()
                    prev_consumption = lastValues[2]
                    prev_time = mktime(lastValues[0].timetuple())

                    print("Last database entry: {} kWh on {} GMT ({})".format(
                        prev_consumption, lastValues[0].strftime('%A %d-%m-%Y, %H:%M:%S'), prev_time))
                    
                    # Start up the server to expose the metrics.
                    port = int(os.environ.get("PROMETHEUS_PORT", "9400"))
                    print("PROMETHEUS_PORT={}".format(port))
                    server, t = start_http_server(port)
                    grid_energy_draw = Gauge('grid_energy_draw', 'Total sum of energy consumption from the grid in Wh')
                    grid_energy_feed = Gauge('grid_energy_feed', 'Total sum of energy fed to the grid in Wh')

                    prev_time = 0

                    try:
                        while True:
                            obis_values = meter.readConsumption()
                            # print("{} {}".format(datetime.now().strftime('%A %d-%m-%Y, %H:%M:%S'), obis_values))

                            if obis_values:
                                if '1.8.0' in obis_values:
                                    value = obis_values['1.8.0']
                                    grid_energy_draw.set(value)

                                    mqtt_client.publishObject(
                                        "home/energy/power/consumption", value / 1000, "kWh", retain=True)
                                    mqtt_client.publish(
                                        "home/energy/power/consumption/value", '{0:0.1f}'.format(value / 1000), retain=True)


                                if '2.8.0' in obis_values:
                                    value = obis_values['2.8.0']
                                    grid_energy_feed.set(value)

                                    mqtt_client.publishObject(
                                        "home/energy/power/feed", value / 1000, "kWh", retain=True)
                                    mqtt_client.publish(
                                        "home/energy/power/feed/value", '{0:0.1f}'.format(value / 1000), retain=True)

                                if time() - prev_time >= 60 * 5:  # every 5 minutes
                                    if '1.8.0' in obis_values:
                                        value = obis_values['1.8.0']
                                        cur.execute("INSERT INTO consumption (timestamp, type, value) VALUES (%s, 'strom', %s)", (
                                            datetime.now(timezone.utc), value / 1000))  # convert Wh to kWh
                                    if '2.8.0' in obis_values:
                                        value = obis_values['2.8.0']
                                        cur.execute("INSERT INTO consumption (timestamp, type, value) VALUES (%s, 'einspeisung', %s)", (
                                            datetime.now(timezone.utc), value / 1000))  # convert Wh to kWh
                                    conn.commit()
                                    prev_time = time()
                            else:
                                print("No obis values received from meter")
                            sleep(5)

                    except KeyboardInterrupt:
                        print("cancel")
                        server.shutdown()
                        t.join()
