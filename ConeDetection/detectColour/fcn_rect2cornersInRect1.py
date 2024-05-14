def rect2cornersInRect1(rect1points,rect1area,rect2points):
    #given the four corner points of each rectangle
    #check if the corners of rectangle 2 lie in rectangle 1 
    
    A = rect1points[0]
    B = rect1points[1]
    C = rect1points[2]
    D = rect1points[3]
    Ax = A[0]
    Ay = A[1]
    Bx = B[0]
    By = B[1]
    Cx = C[0]
    Cy = C[1]
    Dx = D[0]
    Dy = D[1]
    
    intersection = False
    
    for i in range(0,4): #check for all corner points of rectangle 2 if they are inside rectangle 1#
        P = rect2points[i]
        Px = P[0]
        Py = P[1]
        areaSum = 0
        areaSum = computeAreaTriangle(Px,Py,Ax,Ay,Bx,By)+computeAreaTriangle(Px,Py,Bx,By,Cx,Cy)+computeAreaTriangle(Px,Py,Cx,Cy,Dx,Dy)+computeAreaTriangle(Px,Py,Dx,Dy,Ax,Ay)
        #areaSum = rect1area means that P is inside the rectangle or on the borderline
        if areaSum <= 1.05*rect1area: #evtl round?
            intersection = True
    
    return intersection
        

def computeAreaTriangle(Ax,Ay,Bx,By,Cx,Cy):
    areaOfTriangle = abs( (Bx * Ay - Ax * By) + (Cx * By - Bx * Cy) + (Ax * Cy - Cx * Ay) ) / 2
    return areaOfTriangle