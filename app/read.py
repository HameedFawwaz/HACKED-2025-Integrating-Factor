import serial

arduino = serial.Serial(port="COM9", baudrate=9600, timeout=0.1)

while True:
    data = arduino.readline().decode("utf-8")
    print(data.split(","))