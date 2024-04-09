# ------------------------------------- AVAILABLE FUNCTIONS --------------------------------#
# qube.setRGB(r, g, b) - Sets the LED color of the QUBE. Color values range from [0, 999].
# qube.setMotorSpeed(speed) - Sets the motor speed. Speed ranges from [-999, 999].
# qube.setMotorVoltage(volts) - Applies the given voltage to the motor. Volts range from (-24, 24).
# qube.resetMotorEncoder() - Resets the motor encoder in the current position.
# qube.resetPendulumEncoder() - Resets the pendulum encoder in the current position.

# qube.getAnglePosition() - Returns the cumulative angular positon of the motor.
# qube.getPendulumPosition() - Returns the cumulative angular position of the pendulum.
# qube.getMotorRPM() - Returns the newest rpm reading of the motor.
# qube.getMotorCurrent() - Returns the newest reading of the motor's current.
# ------------------------------------- AVAILABLE FUNCTIONS --------------------------------#

from QUBE import *
from fullStateFeedback import *
from logger import *
from serialReader import *
from zeroOrderHold import *
from com import *
from liveplot import *
import time
import threading


A = np.array([[0, 1], [0, -10.05]])
B = np.array([[0], [239.25]])
C = np.array([1,0])# System matrices
K = np.array([[0.00619, 0.00127]])
Ki = 0.0305
#integralError= 0
L = np.array([[17.0], [140.3025]])  # Observer gain matrix
u = 0 #input



# Replace with the Arduino port. Can be found in the Arduino IDE (Tools -> Port:)
port = "COM10"
#setptRedrPrt = "COM8" #Com port for reading the user input for the setpoint for motor position

baudrate = 115200
qube = QUBE(port, baudrate)

# Resets the encoders in their current position.
qube.resetMotorEncoder()
qube.resetPendulumEncoder()

# Enables logging - comment out to remove
enableLogging()

#t_last = time()


lastRunTime = 0
m_target = 0
p_target = 0
pid = PID()
zoh = ZeroOrderHold(0.05)
SSController = StateSpaceController(A, B, C, K, Ki, L)




#serialSPRedr = serial.Serial(setptRedrPrt, 9600, timeout=1)

#controlOutput = SSController.runControlLoop(qube.getMotorAngle(), 0, 0.0001)

def control(data, lock):
    global m_target, p_target, pid, controlOutput, zoh, lastRunTime
    while True:
        # Updates the qube - Sends and receives data
        qube.update()

        # Gets the logdata and writes it to the log file
        logdata = qube.getLogData(m_target, p_target)
        save_data(logdata)

        # Multithreading stuff that must happen. Dont mind it.
        with lock:
            doMTStuff(data)

        # Get deltatime
        #dt = getDT(SSController.control())

        #if zoh.ckeckTimer(time.perf_counter()):
            #controlOutput = SSController.runControlLoop(qube.getMotorPosition(), seriServoSpReader(serialSPRedr), time.perf_counter()-lastRunTime)
        controlOutput = SSController.runControlLoop(qube.getMotorAngle(), 0, time.perf_counter() - lastRunTime)
        lastRunTime = time.perf_counter()

        ### Your code goes here
        qube.setMotorVoltage(controlOutput)
        #O


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
