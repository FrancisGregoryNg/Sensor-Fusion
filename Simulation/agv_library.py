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
            self.vx = self.vx + dv
        else:                           #else (if the difference in desired vs actual is small) just set the velocity directly
            self.vx = newVx
            
        if (abs(newVx - self.vx) > dv):
            self.vy = self.vy + dv
        else:
            self.vy = newVy
            
        self.encoderX += self.omegaX * dt
        self.encoderY += self.omegaY * dt
            
    def readEncoder(self):              #encoder counts every 0.5 degrees, so convert from rad to deg, then multiply by 2
        countsX = 2 * self.encoderX * 180 / 3.14159265359
        countsY = 2 * self.encoderY * 180 / 3.14159265359
        
    def readIEEE(self):
        return self.ieee
        
    def readActual(self);
        return (self.x, self.y)
            
roboman = Vehicle(0, 0, 0, 0, 0.05, 1, 0.5);
