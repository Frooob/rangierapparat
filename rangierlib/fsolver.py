from scipy.optimize import fsolve
import math
 

def ecke_vorne_rechts(x,y,a, f = .33, w = .19):
    """ Ergibt mit Modellbeispiel (39.497, 13.091) ist korrekt."""
    xn = x + f * math.cos(a) + (w/2) * math.sin(a)
    yn = y - (w/2) * math.cos(a) + f * math.sin(a)
    return (xn, yn)


def ecke_hinten_links(x, y, a, b=.05, w=.19):
    """ Ergibt mit Modellbeispiel (1.942, 19.641) ist korrekt."""
    xn = x - b * math.cos(a) - (w/2) * math.sin(a)
    yn = y + (w/2) * math.cos(a) - b * math.sin(a) 
    return (xn, yn)


def radius_ecke_vorne_rechts(r = .6588, f=.33, w=.19):
    """ Ergibt mit Modellbeispiel 82.2869 ist korrekt."""
    pos_ecke_vorne_rechts = ecke_vorne_rechts(0,0,0, f, w)
    pos_auto_drehkreis_mittelpunkt = (0, r)
    return math.dist(pos_ecke_vorne_rechts, pos_auto_drehkreis_mittelpunkt)


def radius_ecke_hinten_links(r=.6588, b=.05, w=.19):
    """ Ergibt mit Modellbeispiel 75.546 ist korrekt."""
    pos_ecke_hinten_links = ecke_hinten_links(0, 0, 0, b, w)
    pos_auto_drehkreis_mittelpunkt = (0, -r)
    return math.dist(pos_ecke_hinten_links, pos_auto_drehkreis_mittelpunkt)


def alpha_verschiebung_vorne_rechts(r, f, w):
    """ Ergibt mit Modellbeispiel 23.643 ist korrekt."""
    pos_ecke_vorne_rechts = ecke_vorne_rechts(0, 0, 0, f, w)
    pos_auto_drehkreis_mittelpunkt = (0, r)
    radius_vorne_rechts = radius_ecke_vorne_rechts(r,f,w)
    alpha_vorne_rechts = math.acos(
        (pos_ecke_vorne_rechts[0] - pos_auto_drehkreis_mittelpunkt[0]) / radius_vorne_rechts)
    alpha_vorne_rechts = 2*math.pi - alpha_vorne_rechts
    return alpha_vorne_rechts - 1.5*math.pi



def alpha_verschiebung_hinten_links(r, b, w):
    """ Ergibt mit Modellbeispiel -5.1 ist korrekt."""
    pos_ecke_hinten_links = ecke_hinten_links(0, 0, 0, b, w)
    pos_auto_drehkreis_mittelpunkt = (0, r)
    radius_hinten_links = radius_ecke_hinten_links(r, b, w)
    alpha_hinten_links = math.acos(
        (pos_ecke_hinten_links[0] - pos_auto_drehkreis_mittelpunkt[0]) / radius_hinten_links)
    alpha_hinten_links = 2*math.pi - alpha_hinten_links
    return alpha_hinten_links - 1.5*math.pi


def center_of_circle(r, initial_a, p_car, direction):
    if direction == "forward":
        initial_a = initial_a + 1.5 * math.pi
    else:
        initial_a = initial_a + 0.5 * math.pi

    x_mid = p_car[0] - r * math.cos(initial_a)
    y_mid = p_car[1] - r * math.sin(initial_a)
    return x_mid, y_mid


def position_after_rotation(a, r, initial_a = 0, p_car = (0,0), direction="forward"):
    """
    Berechnet die Position auf einer Kreisbahn nach einer Rotation.
    Die Rotationsrichtung ist nach vorne links.
    """
    
    # Erstmal den Mittelpunkt von dem Kreis ausrechnen, aus dem initialen Winkel und p_car
    x_mid, y_mid = center_of_circle(r, initial_a, p_car, direction)
    # print(f"center: {(x_mid, y_mid)}")
    
    if direction == "forward":
        initial_a = initial_a + 1.5* math.pi
    else:
        initial_a = initial_a + 0.5*math.pi

    x = r * math.cos(a + initial_a) + x_mid
    y = r * math.sin(a + initial_a) + y_mid
    
    return x,y


def dist_func_to_minimize(r_car, r_outer_edge_car, p_target, initial_a = 0, p_car = (0,0)):
    """
    Bekommt die Werte der drei Parameter und gibt die verkettete
    Funktion von pos und dist zurück, die nur noch a als Parameter braucht. 
    """
    def return_func(a):
        p_car_after_rotation = position_after_rotation(a, r_car, initial_a, p_car, "backward")
        center_of_upper_circle = center_of_circle(
            r_car, initial_a + a, p_car_after_rotation, "forward")
        d = math.dist(center_of_upper_circle, p_target) - r_outer_edge_car
        return d
    return return_func


"""
Realistisches Beispiel:
Auto steht anfangs bei (5.238,10.732)
Radius des Autos liegt bei: 65.88
Radius der vorderen rechten Ecke liegt bei 82.2287
Das auto ist bereits um 20° (0.1111111111111111 * pi) gedreht.

Der Punkt den wir erreichen wollen liegt bei: (50.176, 32.447)

Frage: Wie weit müssen wir noch nach hinten drehen, bis wir nach vorne mit der Schnauze vorbeikommen?
"""

if __name__ == "__main__":
    car_pos = (5.238, 10.732)
    target_pos = (50.176, 32.447)
    initial_alpha = 0.1111111111*math.pi
    car_rad = 65.88
    outer_front_car_rad = 82.2287

    rotated_alpha = initial_alpha
    """
    Erstes Händisches Beispiel: 
    Siehe in der Fusion Datei Test_nach_hinten_fahren
    Dort wird der Sachverhalt modelliert wo wir landen, wenn das Auto bereits 20° eingedreht ist und weitere 20° nach hinten fährt. 
    Zunächst ist dann der Mittelpunkt vom Auto Bei (-14.576,-0.708) und der Mittelpunkt vom oberen Kreis bei (-56.28, 48.99). 
    """
    pos_after_back_rotation = position_after_rotation(
        rotated_alpha, car_rad, initial_alpha, car_pos, "backward")
    print(f"Position after back rotation: {pos_after_back_rotation}")
    initial_uper_circle_middle = center_of_circle(
        car_rad, initial_alpha + rotated_alpha, pos_after_back_rotation, "forward")
    print(f"initial mid of upper circle should be: {initial_uper_circle_middle}")

    """
    Erstes Optimierungsbeispiel:
    Wenn wir nun unsere Funktion optimieren möchten, dann sollte das Auto noch ca. 2.5° nach hinten gedreht werden, bevor man mit der Schnauze vorbei fahren könnte.
    Dies soll der minimizer nun selbst rausfinden:
    """

    example_function = dist_func_to_minimize(
        car_rad, outer_front_car_rad, target_pos, initial_alpha, car_pos)
    solutions = fsolve(example_function, 0)
    print(f"Das Auto muss noch {math.degrees(solutions[0])}° nach hinten fahren.")
