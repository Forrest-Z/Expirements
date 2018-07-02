from math import *

def tang_from_point(circle, point): # circle = [[α, β], r] ; point [xP, yP]
    #your code here
    a,b,r = circle[0][0],circle[0][1],circle[1]
    xP,yP = point[0],point[1]
    tan1 = float(yP-b)/float(xP-a)
    dis2 = float(yP-b) **2 + float(xP-a)**2
    len2 = sqrt(dis2-r**2)
    tan2 = float(r)/float(len2)
    m1 = (tan1-tan2)/(1+tan1*tan2)
    m2 = (tan1+tan2)/(1-tan1*tan2)
    n1 = yP - m1*xP
    n2 = yP - m2*xP
    return [[round(m1,4),round(n1,4)], [round(m2,4),round(n2,4)]] # parameters of tangent lines 
                                # t1) y = m1.x + n1
                                # t2) y = m2.x + 

circle = [[-3.0, 0.0], 3.0] 
point =  [-5.0, 0.0]

tang_from_point(circle, point)