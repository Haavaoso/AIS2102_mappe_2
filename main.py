# ------------------------------------- AVAILABLE FUNCTIONS --------------------------------#
# qube.setRGB(r, g, b) - Sets the LED color of the QUBE. Color values range from [0, 999].
# qube.setMotorSpeed(speed) - Sets the motor speed. Speed ranges from [-999, 999].
# qube.setMotorVoltage(volts) - Applies the given voltage to the motor. Volts range from (-24, 24).
# qube.resetMotorEncoder() - Resets the motor encoder in the current position.
# qube.resetPendulumEncoder() - Resets the pendulum encoder in the current position.

# qube.getMotorPosition() - Returns the cumulative angular positon of the motor.
# qube.getPendulumPosition() - Returns the cumulative angular position of the pendulum.
# qube.getMotorRPM() - Returns the newest rpm reading of the motor.
# qube.getMotorCurrent() - Returns the newest reading of the motor's current.
# ------------------------------------- AVAILABLE FUNCTIONS --------------------------------#

from QUBE import *
from logger import *
from serialReader import *
from com import *
from liveplot import *
from time import time
import numpy as np
import threading
import math






# Replace with the Arduino port. Can be found in the Arduino IDE (Tools -> Port:)
port = "COM5"
# setptRedrPrt = "COM8" #Com port for reading the user input for the setpoint for motor position

baudrate = 115200
qube = QUBE(port, baudrate)

# Resets the encoders in their current position.
qube.resetMotorEncoder()
qube.resetPendulumEncoder()

# Enables logging - comment out to remove
enableLogging()

t_last = time()

m_target = 0
p_target = 90
pid = PID()


#serialSPRedr = serial.Serial(setptRedrPrt, 9600, timeout=1)


def control(data, lock):
    global m_target, p_target, pid, iii
    start_time = time()  # Get the start time

    while True:
        # Updates the qube - Sends and receives data
        qube.update()

        # Gets the logdata and writes it to the log file
        logdata = qube.getLogData(m_target, p_target)
        save_data(logdata)
        current_time = time()  # Get the current time
        elapsed_time = current_time - start_time  # Calculate elapsed time
        sine_value = math.sin(elapsed_time)

        # Scale the sine value to match the motor voltage range
        # This example assumes a full range (-24 to 24 volts), adjust as necessary
        motor_voltage = sine_value * 3

        # Multithreading stuff that must happen. Dont mind it.
        with lock:
            doMTStuff(data)

        # Get deltatime
        dt = getDT()
        #x2 = pid.regulate(p_target, qube.getMotorAngle(), dt)
        x2 = pid.regulate(1000, qube.getMotorRPM(), dt)
        qube.setMotorVoltage(x2)
        #OK motherfucker

        #seriServoSpReader(serialSPRedr)
        # print(f"this is the motor setpoint position: {seriServoSpReader(serialSPRedr)}")
        #print(f"This is the motor position: {qube.getMotorAngle()} and Pid {x2}")


def getDT():
    global t_last
    t_now = time()
    dt = t_now - t_last
    t_last += dt
    return dt


def doMTStuff(data):
    packet = data[7]
    pid.copy(packet.pid)
    if packet.resetEncoders:
        qube.resetMotorEncoder()
        qube.resetPendulumEncoder()
        packet.resetEncoders = False

    new_data = qube.getPlotData(m_target, p_target)
    for i, item in enumerate(new_data):
        data[i].append(item)


if __name__ == "__main__":
    _data = [[], [], [], [], [], [], [], Packet()]

    lock = threading.Lock()
    thread1 = threading.Thread(target=startPlot, args=(_data, lock))
    thread2 = threading.Thread(target=control, args=(_data, lock))
    thread1.start()
    thread2.start()
    thread1.join()
    thread2.join()
