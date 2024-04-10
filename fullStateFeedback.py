import numpy as np

# System parameters





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
        # NEW SELF.XHAT
        self.xHat = self.xHat + dt * (self.A @ self.xHat + self.B * self.u + self.L * error)

        self.integralWindup(reference, y, dt)

    def calculateControlInput(self):
        # The control law: u = -float(K * x_hat - Ki * integral_error)
        self.u = -float((self.K @ self.xHat - self.Ki * self.integralError))

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


    
    

