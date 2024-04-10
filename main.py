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
import math

A = np.array([[0, 1], [0, -10.05]])
B = np.array([[0], [239.25]])
C = np.array([1,0])# Here we
K = np.array([[0.00619, 0.00127]])
Ki = 0.01
#integralError= 0
L = np.array([[17.0], [4.3]])  # Observer gain matrix
u = 0 #input

#THIS IS THE BEGINNING OF THE SECTION WHERE WE CAN ENABLE/DISABLE DIFFERENT FUNCTIONS OF THE PROGRAM
ENABLE_DATA_RECORDING = False


#THIS IS THE END OF THE SECTION WHERE WE CAN ENABLE/DISABLE DIFFERENT FUNCTIONS OF THE PROGRAM




port = "COM10"
setptRedrPrt = "COM8" #Com port for reading the user input for the setpoint for motor position

baudrate = 115200
qube = QUBE(port, baudrate)

# Resets the encoders in their current position.
qube.resetMotorEncoder()
qube.resetPendulumEncoder()

# Enables logging - comment out to remove
enableLogging()

#t_last = time()

iteration = 0
lastRunTime = 0
m_target = 0
p_target = 0
pid = PID()
zoh = ZeroOrderHold(0.05)
SSController = StateSpaceController(A, B, C, K, Ki, L)



fieldnames = ["iteration","observerX1AngularPosition", "observerX2AngularVelocity", "realAngularPosition", "motorAngularSpeed"]
output_file = f'observer_plots/observerData.csv'
with open(output_file, 'w') as csv_file:
    csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    csv_writer.writeheader()

def saveObserverData(observerAngularPosition, observerAngularVelocity, qube):
    global iteration
    output_file = f'observer_plots/observerData.csv'
    with open(output_file, 'a') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        info = {
            "iteration": iteration,
            "observerX1AngularPosition": observerAngularPosition,
            "observerX2AngularVelocity": observerAngularVelocity,
            "realAngularPosition": qube.getMotorAngle(),
            "motorAngularSpeed": ((qube.getMotorRPM())*(math.pi)*(2/60)),
        }
        csv_writer.writerow(info)
        iteration += 1




def control(data, lock):
    global m_target, p_target, pid, controlOutput, SSController, zoh, lastRunTime
    while True:
        # Updates the qube - Sends and receives data
        qube.update()

        # Gets the logdata and writes it to the log file
        logdata = qube.getLogData(m_target, p_target)
        save_data(logdata)

        with lock:
            doMTStuff(data)

        # Get deltatime
        #dt = getDT(SSController.control())

        if ENABLE_DATA_RECORDING:
            # ---------------------- THIS IS SAVING THE STATES --------------------------
            saveObserverData(SSController.getObsState1()[0], SSController.getObsState2()[0], qube)
            # ----------------- THIS IS SAVING THE STATES IN THE CSV FILE --------------------------


        #controlOutput = SSController.runControlLoop(qube.getMotorAngle(), seriServoSpReader(serialSPRedr), time.perf_counter()-lastRunTime)
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
