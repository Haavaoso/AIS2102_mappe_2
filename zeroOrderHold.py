
class ZeroOrderHold:
    def __init__(self, ZOHholdTime):
        self.lastValue = 0
        self.currOutStartTime  =
        self.holdTime = ZOHholdTime
    def update(self, newValue):
        self.last_value = newValue

    def output(self):
        return self.lastValue

    def ckeckTimer(self, currentTime, newValue):
        if (currentTime > (self.ZOHholdTime + self.currOutStartTime)):
            self.last_value = newValue
        else:



# Initiating
# zoh = ZeroOrderHold()