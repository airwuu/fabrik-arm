import numpy as np
import csv

def update_coords(vectors, filename='coords.csv'):
    """
    Converts an array of Vector objects to a CSV file.
    """
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y'])  # Write the header
        for vector in vectors:
            writer.writerow([vector.getx(), vector.gety()])

class Vector:
    def __init__(self, x, y):
        self.vector = np.array([x, y])
    def magnitude(self):
        return np.linalg.norm(self.vector)
    def unit(self):
        mag = self.magnitude()
        if mag == 0:
            return Vector(0, 0)
        return Vector(self.vector[0] / mag, self.vector[1] / mag)
    def __sub__(self, other):
        return Vector(self.vector[0] - other.vector[0], self.vector[1] - other.vector[1])
    def __mul__(self, scalar):
        return Vector(self.vector[0] * scalar, self.vector[1] * scalar)
    def __repr__(self):
        return f"Vector({self.vector[0]}, {self.vector[1]})"
    def __add__(self, other):
            return Vector(self.vector[0] + other.vector[0], self.vector[1] + other.vector[1])
    def getx(self):
        return (self.vector[0])
    def gety(self):
        return (self.vector[1])

#Notes: 490 by 490 bounding box, 14in by 14in, 1in = 35px, 4-4-6 Lengths, equating to 140-140-210px
class Chain:
    def __init__(self):
        #initialize the chain object to standupright
        self.p0 = Vector(3, 0)
        self.p1 = Vector(13, 139.64)
        self.p2 = Vector(23, 279.28)
        self.p3 = Vector(123, 404.18)
        self.l1 = 140
        self.l2 = 140
        self.l3 = 210
        #Default goal is straight up
        self.goal=Vector(318, 100)

    def validateBounds(self, l1, l2, l3, p0, goal): #Checks whether the point is physically attainable by the robot:
        mag = (goal - p0).magnitude()
        total_length = l1+l2+l3
        # print(mag, total_length)
        return total_length >= mag
    def validateAngles(self, angle): #helper function to make sure all angles are legal before sending it off to destroy the robot arm
        if angle > 180:
            return 180
        elif angle < 0:
            return 0
        else:
            return angle

    def calculateAngles(self):
        # A kind of dumb way to calculate absolute angles for each motor
        absolute_angles = [0,0,0]
        unit_vector1 = (self.p1 - self.p0).unit()
        absolute_angles[0]= int(((np.arctan2(unit_vector1.gety(), unit_vector1.getx())*180/np.pi)))
        unit_vector2 = (self.p2 - self.p1).unit()
        absolute_angles[1]=int(((np.arctan2(unit_vector2.gety(), unit_vector2.getx())*180)/np.pi))
        unit_vector3 = (self.p3 - self.p2).unit()
        absolute_angles[2]=int(((np.arctan2(unit_vector3.gety(), unit_vector3.getx())*180)/np.pi))
        # Convert Absolute Angles to Relative for motors to use
        relative_angles = [0,0,0]
        relative_angles[0] = self.validateAngles(absolute_angles[0])
        relative_angles[1] = self.validateAngles(90 - absolute_angles[0] + absolute_angles[1])
        relative_angles[2] = self.validateAngles(90 - absolute_angles[1] + absolute_angles[2])
        # print("Absolute: ",absolute_angles)
        # print("Relative: ",relative_angles)
        return relative_angles

    def newPoint(self, p2, p1, length): #quick helper function to pump out new points
        unit_vector = (p2 - p1).unit()
        new_point = (unit_vector * length) + p1
        return new_point
    def forwardKinematics(self, p0prime, p1prime, p2prime, p3prime): #Forward version where we begin at P0 and work towards P3
        # print("Backward Points: ", p0prime, p1prime, p2prime, p3prime)
        self.p0 = self.p0
        self.p1 = self.newPoint(p1prime, self.p0, self.l1)
        self.p2 = self.newPoint(p2prime, self.p1, self.l2)
        self.p3 = self.newPoint(p3prime, self.p2, self.l3)
        # print("Forward Points: ", self.p0, self.p1 ,self.p2 ,self.p3)
        self.calculateAngles()
    def backwardKinematics(self): #Begin at the goal and move back to P0
        p3prime = self.goal
        p2prime = self.newPoint(self.p2, p3prime, self.l3)
        p1prime = self.newPoint(self.p1, p2prime, self.l2)
        p0prime = self.newPoint(self.p0, p1prime, self.l1)
        self.forwardKinematics(p0prime,p1prime,p2prime,p3prime)
    def fabrik(self, goal): # Uses forwards and backwards kinematics to iterate towards the goal. Sets margin of error
        self.goal = goal
        if (self.validateBounds(self.l1, self.l2, self.l3, self.p0, self.goal)):
            # do fabrik
            while ((self.goal - self.p3).magnitude() > 0.1): #margin of error/end condition
                # print((self.goal - self.p3).magnitude())
                self.backwardKinematics()
                # self.forwardKinematics() I changed it so that backwards kinematics will automatically call forwards
            update_coords([self.p0,self.p1,self.p2,self.p3])
            return self.calculateAngles()
        else:
            print("out of bounds")



# arm = Chain()
# arm.fabrik()
# arm.calculateAngles()
