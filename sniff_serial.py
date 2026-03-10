import serial, time

try:
    s = serial.Serial('/dev/ttyACM0', 115200, timeout=3)
    time.sleep(2)
    d = s.read(500)
    print("bytes received:", len(d))
    print("raw:", repr(d))
    if d:
        try:
            print("text:", d.decode('utf-8', errors='replace'))
        except:
            pass
    s.close()
except Exception as e:
    print("Error:", e)
