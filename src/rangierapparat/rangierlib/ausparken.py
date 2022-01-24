import numpy as np
from .circle_intersections import circle_intersection
import math

from .fsolver import center_of_circle


def delta_x_und_y_um_rechts_gerade_zu_lenken(alpha, car_rad):
    """
    Bekommt ein Alpha, wie das Auto momentan steht und berechnet, wie weit es noch in der y-Achse nach oben geht, wenn es gerade gelenkt wird. 
    """
    # es geht von alpha + 90° bis 90°
    # y = sin a * r
    y1 = math.sin(alpha+0.5*math.pi) * car_rad
    y2 = math.sin(0.5*math.pi) * car_rad
    yd = y2-y1
    
    x1 = math.cos(alpha+0.5*math.pi) * car_rad
    x2 = math.cos(0.5*math.pi) * car_rad
    xd = x2-x1

    return xd,yd

def ausparkvorgang(car_pos, alpha, car_rad, p_VA, f, w, b, y_final, actions):
    """
    Das Auto steht nun an einer Position, bei der es voll nach links eingeschlagen an dem vorderen Auto vorbeikommen würde. 
    Der Ausparkvorgang sieht wie folgt aus:
    
    1. Fahre so lange voll links eingeschlagen nach vorne, bis der rechte Wendekreis zu p_VA mindestens/genau einen Abstand von w/2 hat.
    1.5 Berechne, wie viel y Wir durch das geradelenken bei gegebenem Winkel gemacht werden
    2. Fahre dann so lange gerade, bis man dann wenn man nach rechts einschlägt auf der richtigen Höhe landen wird.
    3. Lenke gerade. 
    Jetzt kann es sein, dass der Winkel des Autos zu steil ist. 
    - in dem Fall: Verkackt, kann jemand anders machen. 
    
    
    Wie kriegen wir es nun hin, dass dieser Abstand jetzt richtig ist?
    - Ich brauche ja diesmal keinen Abstand zu einem Punkt, sondern einem Kreis. 
    - Es geht ja schon auch darum, solange zu drehen, bis man nur noch einen Schnittpunkt mit dem 
    
    zu 1. 
    Zunächst brauchen wir die den Mittelpunkt vom rechten Wendekreis in Abhängigkeit vom Drehwinkel -> gibt es ja schon. center_of_circle("backward")
    
    Erhöhe dann solange den den Drehwinkel (maximal bis 90° so), bis der return von circle_intersection == 2 ist. 
    
    
    """
    
    # 1. Finde den Winkel, der nach links abgefahren werden muss (append to actions)
    new_car_pos, new_alpha = finde_links_fahr_winkel_beim_ausparken(car_pos, alpha, car_rad, p_VA, w)
    
    # TODO: Handle None (wird momentan nen error schmeißen, wenn nix rauskommt)
    
    
    # Berechne die Position die das Auto haben wird nachdem der Winkel gefahren ist
    actions.append(("vorwärts", "links", new_alpha-alpha))
    
    # 2. Berechne, wie weit nach x und y gefahren würde, beim gerade lenken in der Rechtskurve.
    dx, dy = delta_x_und_y_um_rechts_gerade_zu_lenken(new_alpha, car_rad)
    
    # 3. Berechne aus der letzten Position und dem geradelenken, wie weit noch geradeaus gefahren werden muss (welches +x resultiert)
    y_missing = y_final - (new_car_pos[1] + dy)
    
    if y_missing < 0:
        print("Leider kann in diese Parklücke nicht gefahren werden.")
        return
        
    hypothenuse_geradeaus_zu_fahren = y_missing/math.sin(new_alpha)
    x_missing = y_missing/math.tan(new_alpha)
    
    # 4. Füge die letzten Anweisungen hinzu und fertig ist der Ausparkprozess
    actions.append(("vorwärts", "geradeaus", hypothenuse_geradeaus_zu_fahren))
    actions.append(("vorwärts", "rechts", new_alpha))
    x_final = new_car_pos[0] + dx + x_missing
    
    return (x_final, y_final)
    
    
    ...


def finde_links_fahr_winkel_beim_ausparken(car_pos, alpha, car_rad, p_VA, w):

    lower_circle_mid = center_of_circle(car_rad, alpha, car_pos, "backward")

    first_intersection = circle_intersection(
        p_VA[0], p_VA[1], w/2, lower_circle_mid[0], lower_circle_mid[1], car_rad)
    
    upper_circle_center = center_of_circle(
        car_rad, alpha, car_pos, "forward")

    # print(f"first intersection {first_intersection}")
    if len(first_intersection) != 2:
        print("Dangerzone! Es sollte am Anfang schon zwei Überschneidungen geben")
    for possible_alpha in np.linspace(alpha, 0.5*math.pi, 100):

        """
        1. calculate new car pos with new alpha
        2. calculate new lower_circle_mid with new alpha and car pos
        """
        x_new = upper_circle_center[0] + car_rad * \
            math.cos(1.5*math.pi + possible_alpha)
        y_new = upper_circle_center[1] + car_rad * \
            math.sin(1.5*math.pi + possible_alpha)
        car_pos = (x_new, y_new)

        lower_circle_mid = center_of_circle(
            car_rad, possible_alpha, car_pos, "backward")
        # print(f"upper circle mid: {upper_circle_center} car pos: {car_pos} lower circle mid: {lower_circle_mid}")

        intersection = circle_intersection(
            p_VA[0], p_VA[1], w/2, lower_circle_mid[0], lower_circle_mid[1], car_rad)
        if len(intersection) == 1:
            # print(f"FOUND IT!{possible_alpha}")
            return car_pos, possible_alpha
        
    print("Konnte nicht den richtigen Winkel zum Ausparken finde. Ab jetzt wirds holprig (ich garantiere für nichts mehr).")
    ...


if __name__ == "__main__":
    p_VA = (50.176, 32.447)
    w = 19
    car_rad = 65.88
    initial_car_pos = (17.437, 24.8)
    
    initial_alpha = math.radians(33.6)
    print(f"initial alpha: {initial_alpha}")


    finde_links_fahr_winkel_beim_ausparken(initial_car_pos, initial_alpha, car_rad, p_VA, w)
