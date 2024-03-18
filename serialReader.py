import serial
import time
#from main import setptRedrPrt







def seriServoSpReader(setptRedrPrt):
    # Change 'COM3' to the port your Arduino is connected to.
    # com8 is the top left port of HKU laptop
    # wait for the serial connection to initialize

    while True:
        if setptRedrPrt.in_waiting > 0:
            line = setptRedrPrt.readline().decode('utf-8').rstrip()
            try:
                # Convert the string to a float and print
                val = float(line)
                print(f"Received float value: {val}")
                return val



            except ValueError:
                # In case the conversion fails, which can happen if the data is incomplete or corrupted.
                print(f"Failed to convert to float: {line}")

                return -180