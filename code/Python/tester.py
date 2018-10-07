import gimbal as gb
import serial.tools.list_ports as lp


comports = lp.comports()

for port in comports:
    print("Port found: " + port.description)
    try:
        gimbal = gb.Gimbal(port.device)
    except Exception as e:
        print(str(e))
        pass
    else:
        break

major, minor = gimbal.get_version()

print("MAJOR: " + major)
print("MINOR: " + minor)
