import time, math
def FDIST_PI(x,y):
    return min( abs(x-y), 2*math.pi - abs(x-y))
def FCOMP_PI(x, y):
    return  (FDIST_PI(x, y) * (-1 if\
        (x>y and x-y < math.pi or x<y and y-x > math.pi)\
        else 1))
class PID():
    der_sample_size = 3    #Sampling size of past slopes
    integ_error = 0.0      #Time integral of error
    der_error = 0.0        #Time derivative of der_error
    val = 0.0              #Latest plugged in value
    result = 0.0           #Latest result
    frame = 0
    past_slopes = []
    def __init__(self, p, i, d):
            self.pgain = p         #proportional gain - K_p*e(t)
            self.igain = i         #integral gain - K_i*integral(e(t))
            self.dgain = d         #differential gain - K_d*d(e(t))
            self.setpoint = 0.0
            self.error = 0.0
            self.past_error = 0.0
            self.slope = 0
            for i in range(0, self.der_sample_size):
                self.past_slopes.append(0.0)
            self.last_update = time.time() * 1000
    def set_setpoint(self, new_setpoint):
        self.setpoint = new_setpoint
    def reset(self):
        self.integ_error = 0.0
        self.error = 0.0
        self.past_error = 0.0
        self.slope = 0
        for i in range(0, self.der_sample_size):
            self.past_slopes[i] = 0.0
        self.last_update = time.time() * 1000
    def integral_error(self):
        self.integ_error = self.integ_error + self.error*self.frame/1000.0;
        return self.integ_error
    def derivative_error(self):
        self.past_slopes[self.slope % self.der_sample_size] = \
            (self.error - self.past_error) if self.frame else 0
        self.slope = self.slope + 1
        self.der_error = 0
        for i in range(0, self.der_sample_size):
            self.der_error = self.der_error + self.past_slopes[i]
        return self.der_error
    def calc(self, current_val):
        self.val = current_val
        self.error = FCOMP_PI(self.setpoint, current_val)
        self.frame = time.time()*1000 - self.last_update
        self.last_update = time.time()*1000;
        self.result = self.pgain*self.error + self.igain*self.integral_error()+\
            self.dgain*self.derivative_error()
        self.past_error = self.error
        return self.result
    def report(self):
        print(3*"\n" + str(self.setpoint) + "\t" + str(self.val) + \
            "\t" + str(self.error))
