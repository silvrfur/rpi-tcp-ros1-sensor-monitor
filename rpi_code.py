#!/usr/bin/env python3

import time
import board
import adafruit_dht
import RPi.GPIO as GPIO
import socket
import json

# GPIO setup
GPIO.setmode(GPIO.BCM)
DHT_PIN = 4
MQ9_PIN = 21
LDR_PIN = 17
LED_PIN = 5

GPIO.setup(MQ9_PIN, GPIO.IN)
GPIO.setup(LDR_PIN, GPIO.IN)
GPIO.setup(LED_PIN, GPIO.OUT)

dhtDevice = adafruit_dht.DHT11(board.D4)

# TCP server setup
HOST = ''  # empty string binds to all interfaces
PORT = 5005
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)
print("Waiting for ROS client to connect...")

conn, addr = server_socket.accept()
print("Connected by", addr)

try:
    while True:
        # Read sensors
        try:
            temperature = dhtDevice.temperature
            humidity = dhtDevice.humidity
        except RuntimeError:
            temperature = None
            humidity = None

        gas_detected = GPIO.input(MQ9_PIN) == GPIO.LOW
        is_dark = GPIO.input(LDR_PIN) == GPIO.LOW

        # Control local LED (optional)
        GPIO.output(LED_PIN, is_dark)

        # Prepare data as JSON
        data = {
            "temperature": temperature,
            "humidity": humidity,
            "gas_detected": gas_detected,
            "is_dark": is_dark
        }

        # Send data over TCP
        message = json.dumps(data)
        conn.sendall(message.encode() + b'\n')
        time.sleep(2)

except KeyboardInterrupt:
    print("Exiting...")
    GPIO.cleanup()
    conn.close()
    server_socket.close()
