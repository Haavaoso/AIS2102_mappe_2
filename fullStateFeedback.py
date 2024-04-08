import numpy as np

# System parameters
A = np.array([[0, 1], [0, -10.05]])
B = np.array([[0], [239.25]])
C = np.array([1,0])# System matrices
K = np.array([[0.2]], [0.0165])  # State feedback matrix
Ki = -0.3511
#integralError= 0
L = [17.9, 15.60]  # Observer gain matrix
u = 0 #input


xHatDot = np.array([[0], [0]])  # Initial
xHat = np.array([[0], [0]])    # Initial

class StateSpaceController:
    def __init__(self, A, B, C, K, Ki, L):
        # Initialize system matrices and gains
        self.A = A
        self.B = B
        self.C = C
        self.K = K
        self.Ki = Ki
        self.L = L

        # Initialize state variables
        self.xHat = np.zeros((A.shape[0], 1))  # Assuming A is square
        self.integralError = 0
        self.u = 0

    def observerUpdate(self, y, dt, reference):
        # Observer update equation
        yHat = self.C @ self.xHat
        error = y - yHat
        self.xHat += dt * (self.A @ self.xHat + self.B * self.u + self.L.reshape(-1, 1) * error)

        # Update integral error
        self.integralError += (reference - y) * dt

    def calculateControlInput(self, reference):
        # Control law: u = -K * x_hat - Ki * integral_error
        self.u = -float(self.K @ self.xHat + self.Ki * self.integralError)

        # Enforce actuator limits
        self.u = max(min(self.u, 24), -24)
        return self.u

    def applyControl(self):
        # Placeholder for actuator code
        pass

    def runControlLoop(self, read_sensors, reference, dt):
        y = read_sensors()  # Read sensor data
        self.observer_update(y, dt, reference)
        u = self.calculate_control_input(reference)
        self.apply_control()


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


    
    

def controller_update():
    # Control law: u = -K * xHatDot
    u = -K @ xHatDot

    if 24 <= u:
     return 23.9
    elif -24 >= u:
     return -23.9

    return u

# Main control loop
while True:
    y = readSensors()  # Read sensor data to get y
    u = controllerUpdate()  # Compute control action
    observerUpdate(y, u)  # Update state estimate
    applyControl(u)  # Apply control action
    # Wait for next iteration (dt) to maintain loop timing
"""