import time
class ZeroOrderHold:
    def __init__(self, ZOHholdTime):
        self.lastValue = 0
        self.currOutStartTime  = time.perf_counter()
        self.holdTime = ZOHholdTime
        self.firstRun = 1

    def ckeckTimer(self, currentTime):
        if self.firstRun:
            self.currOutStartTime = currentTime
            self.firstRun = 0
            returnValue = 1

        elif (currentTime > (self.holdTime + self.currOutStartTime)):
            self.currOutStartTime = currentTime
            returnValue = 1
        else:

            returnValue = 0

        return returnValue
# Initiating
# zoh = ZeroOrderHold()