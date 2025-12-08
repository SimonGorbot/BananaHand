import serial
import time

ser = serial.Serial("/dev/ttyACM0", baudrate=115200, timeout=0.1)

payload = bytes([0x01, 0xFF, 0x42])

while True:
    try:
        ser.write(payload)
        print("sent:", list(payload))
        time.sleep(0.1)
        response = ser.read(64)   # read up to 64 bytes or until timeout

        if response:
            print(f"Received ({len(response)} bytes): {response.hex()}")
        else:
            print("No response")

    except KeyboardInterrupt:
        break

ser.close()
