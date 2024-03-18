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
                # Convert the string to a float and print it
                val = float(line)
                #print(f"Received float value: {val}")
                if (val > 100):
                    print("Output out of range")
                    return 100
                elif (val < -100):
                    print("Output out of range")
                    return -100
                else:
                    return val



            except ValueError:
                # In case the conversion fails,
                # This could happen if the data is incomplete or corrupted.
                print(f"Failed to read the float: {line}")

                return -180