# Importing Libraries
import serial
import time
arduino = serial.Serial(port='/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0', baudrate=9600, timeout=0.1)
num = input("Enter a number: ") # Taking input from user
def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
def read_data():
    data = arduino.readline()
    return data
write_read(num)
while True:
    # num = input("Enter a number: ") # Taking input from user
    data = read_data()
    print(data) # printing the value