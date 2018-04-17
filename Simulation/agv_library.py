import random

class Vehicle:
    def __init__(self, wheelRadius, mass, friction, dt, initialX = 0, initialY = 0, initialVx = 0, initialVy = 0, initialOmegaX = 0, initialOmegaY = 0):
        self.x = initialX
        self.y = initialY
        self.vx = initialVx
        self.vy = initialVy
        self.omegaX = initialOmegaX
        self.omegaY = initialOmegaY
        self.wheelRadius = wheelRadius
        self.mass = mass
        self.friction = friction
        self.dt = dt
        
        self.encoderX = 0
        self.encoderY = 0
        self.ieee = [0, 0]
        
    def setMotor(self, omegaX, omegaY):
        self.omegaX = omegaX
        self.omegaY = omegaY
        
    def updateModel(self):
        newVx = self.wheelRadius * self.omegaX
        newVy = self.wheelRadius * self.omegaY
        
        tractionForce = self.friction * self.mass * 9.80665 #there is only 1 traction force since x and y are assumed to have the same friction coeff.
        acceleration = tractionForce / self.mass
        dv = acceleration * self.dt
        
        if (abs(newVx - self.vx) > dv): #if the absolute difference between desired and actual velocity is greater than dv, increment velocity by dv
            self.vx = self.vx - dv
        else:                           #else (if the difference in desired vs actual is small) just set the velocity directly
            self.vx = newVx
            
        if (abs(newVx - self.vx) > dv):
            self.vy = self.vy - dv
        else:
            self.vy = newVy
        
        self.x += self.vx * self.dt
        self.y += self.vy * self.dt
        
        self.encoderX += self.omegaX * self.dt
        self.encoderY += self.omegaY * self.dt
        
        self.ieee[0] = self.x + random.gauss(0, 1)
        self.ieee[1] = self.y + random.gauss(0, 1)
            
    def readEncoder(self):              #encoder counts every 0.5 degrees, so convert from rad to deg, then multiply by 2
        countsX = 2 * self.encoderX * 180 / 3.14159265359
        countsY = 2 * self.encoderY * 180 / 3.14159265359
        
        return (countsX, countsY)
        
    def readIEEE(self):
        return self.ieee
        
    def readActual(self):
        return (self.x, self.y)

#THE SEGMENT BELOW IS FOR TESTING PURPOSES ONLY==================================================
roboman = Vehicle(0.05, 1, 0.5, 0.01, 0, 0, 1, 10, 0, 0);
#0.05m wheel diameter
#1kg mass
#0.5 friction coefficient
#0.01s timestep
#1 m/s initial velocity x
#10 m/s initial velocity y

for x in range(0, 100):
    print(x, end = " ")
    print(*roboman.readActual())
    #print(*roboman.readIEEE())
    
    roboman.updateModel();