import math


def circle_intersection(x1, y1, r1, x2, y2, r2):
    '''
    @summary: calculates intersection points of two circles
    @param circle1: tuple(x,y,radius)
    @param circle2: tuple(x,y,radius)
    @result: tuple of intersection points (which are (x,y) tuple)
    '''
    # http://stackoverflow.com/a/3349134/798588
    dx, dy = x2-x1, y2-y1
    d = math.sqrt(dx*dx+dy*dy)
    if d > r1+r2:
        # print("#1")
        return [1] # no solutions, the circles are separate
    if d < abs(r1-r2):
        # print("#2")
        return [2]  # no solutions because one circle is contained within the other
    if d == 0 and r1 == r2:
        # print("#3")
        return [3]  # circles are coincident and there are an infinite number of solutions

    a = (r1*r1-r2*r2+d*d)/(2*d)
    h = math.sqrt(r1*r1-a*a)
    xm = x1 + a*dx/d
    ym = y1 + a*dy/d
    xs1 = xm + h*dy/d
    xs2 = xm - h*dy/d
    ys1 = ym - h*dx/d
    ys2 = ym + h*dx/d

    return (xs1, ys1), (xs2, ys2)




if __name__ == "__main__":
    
    """
    Auto ist in der Position, wo es nach links rausfahren könnte, die Frage ist, wie weit es nach links rausfährt. 
    Es sollte so lange rausfahren, bis der rechte Wendekreis des Autos genau an einer Stelle sich mit dem dem Kreis mit Radius w/2 mit Mittelpunkt p_VA befindet.
    p_VA und w/2 sind fix. Genauso r vom Wendekreis     
    """
    
    p_VA = (50.176, 32.447)
    w = 19
    car_rad = 65.88
    
    """
    p1: Das Auto sollte noch zwei Schnittpunkte mit dem Kreis haben
    """
    p1 = (49.684, -32.757)
    
    circle_intersection(p_VA[0], p_VA[1], w/2, p1[0], p1[1], car_rad)
    # Das ist korrekt
    
    """
    p2: Das Auto sollte gerade noch so zwei schnittpunkte haben:
    """
    p2 = (64.549, -22.157)
    circle_intersection(p_VA[0], p_VA[1], w/2, p2[0], p2[1], car_rad)

