import numpy as np

# System parameters



#xHatDot = np.array([[0], [0]])  # Initial
#xHat = np.array([[0], [0]])    # Initial

class StateSpaceController:
    def __init__(self, A, B, C, K, Ki, L):
        # Initialize system matrices and gains
        self.A = A
        self.B = B
        self.C = C
        self.K = K
        self.Ki = Ki
        self.L = L

        # Initializing the state variables
        self.xHat = np.zeros((A.shape[0], 1))
        self.integralError = 0
        self.u = 0

    def getObsState1(self):
        return self.xHat[0]

    def getObsState2(self):
        return self.xHat[1]

    def observerUpdate(self, y, dt, reference):
        # Observer update equation
        yHat = self.C @ self.xHat
        error = y - yHat
        #self.xHat += dt * (self.A @ self.xHat + self.B * self.u + self.L.reshape(-1, 1) * error)
        # NEW SELF.XHAT
        self.xHat = self.xHat + dt * (self.A @ self.xHat + self.B * self.u + self.L * error)

        self.integralWindup(reference, y, dt)



    def calculateControlInput(self, reference):
        # The control law: u = -K * x_hat - Ki * integral_error
        self.u = -float((self.K @ self.xHat - self.Ki * self.integralError))

        # Enforce actuator limits

        #return self.u
    def integralWindup(self, reference, y, dt):
        if (self.integralError > 10 or self.integralError < -10):
            self.integralError = 0
        else:
            self.integralError += (reference - y) * dt
    def control(self):

        return self.u

    def runControlLoop(self, sensorReading, reference, dt):
        y = sensorReading
        self.observerUpdate(y, dt, reference)
        u = self.calculateControlInput(reference)
        print(f"error: {(reference - y)} integralerror: {self.integralError}, output: {u}. sensor reading: {sensorReading}, observer State: {self.xHat} this is DT: {dt}")
        return self.control()


#"""
#def readSensors():
#    """Simulate or interface code to read sensor data."""
#    return np.array([...])
#
#def applyControl(u):
#    """Simulate or interface code to apply control input to actuators."""
#    pass
#
#def observerUpdate(y, u, dt, reference):
#    global xHatDot, xHat, xHatDot, integralError, Ki, u
#    # Observer update equation: xHatDot_dot = A*xHatDot + B*u + L*(y - C*xHatDot)
#    xHatDot[0] = xHatDot[0] + dt*(A[0] @ xHatDot[0] + B[1] @ u + L[1] @ (y - C[0] @ xHatDot[0]))
 #   xHatDot[1] = xHatDot[1] + dt*(A[1] @ xHatDot[1] + B[1] @ u + L[1] @ (y - C[1] @ xHatDot[1]))
#
#    #calculating the error
#    error = xHatDot - reference
#    integralError += (reference - y) * dt
#
#
#    #u = -(K[0]*xHatDot[0] - K[1]*xHatDot[1])/20 #SomethingSomething
#
#    u = -(K @ xHat - Ki * integralError)
#    #updating the state variables
#    #summing the up the angular velocity to get the angular position
#    xHat[0] += dt * xHatDot[0]
#    xHat[1] += dt * xHatDot[1]


    
    

#def controller_update():
#    # Control law: u = -K * xHatDot
#    u = -K @ xHatDot#
#
#    if 24 <= u:
#    return 23.9
#    elif -24 >= u:
#     return -23.9
#
#    return u
#
# Main control loop
#while True:
#    y = readSensors()  # Read sensor data to get y
#    u = controllerUpdate()  # Compute control action
#    observerUpdate(y, u)  # Update state estimate
#    applyControl(u)  # Apply control action
#    # Wait for next iteration (dt) to maintain loop timing
#"""