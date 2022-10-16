import numpy as np
from math import cos,sin,pi,sqrt,pow,atan2, degrees, radians


point = [5, -5 ,1]

degree = degrees(atan2(point[0], point[1]))

print(degree)
theta = radians(-90)
trans_matrix = np.array([[cos(theta), -sin(theta), 0], 
                         [sin(theta),  cos(theta), 0], 
                         [     0    ,      0     , 1]])

det_trans_matrix = np.linalg.inv(trans_matrix)

                                 

       
new_point = trans_matrix.dot([point[0], point[1], 1])
print(new_point)