# from fsolvetest import *
from scipy.optimize.minpack import fsolve
from .fsolver import alpha_verschiebung_hinten_links, dist_func_to_minimize, ecke_vorne_rechts, ecke_hinten_links, center_of_circle, alpha_verschiebung_vorne_rechts, radius_ecke_hinten_links, radius_ecke_vorne_rechts
import math
"""
Bei dem Algorithmus fangen wir wieder mal beim ausparken an.
Das Ziel ist es, mit der Mitte der Hinterachse den Ursprung des Koordinatensystems zu erreichen, welcher genau dort liegt, wo man aus der Parklücke perfekt waagerecht draußen ist. 

Es wird zunächst ein Ausparkvorgang beschrieben. Um statt auszuparken einzuparken muss einfach bei jeder Parkoperation das Vorzeichen vom Winkel und die Fahrtrichtung umgekehrt werden. Außerdem natürlich die gesamte Liste der Operationen.

Wir fangen an, zugemauert in einer winzigen Parklücke, nach hinten hin sehr dicht an dem Hinterauto. Es gibt zunächst zwei Operationen, die sich rekursiv aufrufen und in eine Liste von Anweisungen die später ausgeführt werden sollen ihren Fahrtwinkel + Lenkwinkel + Fahrrichtung schreiben.

Die beiden Operationen heißen: RangierVorwärts und RangierRückwärts. 

RangierVorwärts: 
Schlage voll nach links ein und prüfe, ob wir aus der Lücke rauskämen:
- Falls ja: BREAK: Leite den Ausparkvorgang ein
- Falls nein: Fahr soweit wie möglich nach vorne, RangierRückwärts

RangierRückwärts:
Schlage voll nach rechts ein und berechne, wie weit (alpha) man fahren müsste, um beim nach vorne fahren aus der Lücke kommen zu können.
Berechne, ob es möglich ist, so weit zu fahren, ohne mit der hinteren linken Ecke ans hinterauto anzustoßen. 
- Falls ja: Fahre genau so weit und rangiere vorwärts (sollte dann sofort durchpassen zum Ausparkvorgang)
- Falls nein: Fahre so weit wie es möglich ist, und rangiere vorwärts. 


"""


def rangier_vorwaerts(car_pos, alpha, car_rad, p_VA, p_HA, f, w, b, actions, verbose = False):
    if verbose:
        print(
            f"\nRangiere Vorwärts mit car_pos: {car_pos}, alpha: {alpha} ({math.degrees(alpha)}°)")

    outer_front_car_rad = radius_ecke_vorne_rechts(car_rad, f, w)
    x_vorne_rechts_new, y_vorne_rechts_new, alpha_vorne_rechts_new = pos_und_alpha_vordere_rechte_ecke_so_weit_wie_möglich(
        car_rad, outer_front_car_rad, car_pos, alpha, p_VA[0])

    upper_circle_center = center_of_circle(
        car_rad, alpha, car_pos, "forward")
    # TODO: Das ganze hier doch als Klasse schreiben, die Parameter nerven doch hart...
    # print(y_vorne_rechts_new, p_VA[1])
    if y_vorne_rechts_new >= p_VA[1]:  # Hier muss evtl statt y_vr y_auto hin!
        if verbose:
            print("Yey, man kann ausparken, mache ich jetzt auch.")
        return car_pos, alpha
        ...  # Fahr raus, break!
    else:
        alpha_new = alpha_vorne_rechts_new - \
            alpha_verschiebung_vorne_rechts(car_rad, f, w)
        x_new = upper_circle_center[0] + car_rad * math.cos(alpha_new)
        y_new = upper_circle_center[1] + car_rad * math.sin(alpha_new)
        alpha_new_my_system = alpha_new - 1.5*math.pi
        delta_alpha = alpha_new - 1.5 * math.pi - alpha
        if verbose:
            print(
                f"Nach dem vorwärts rangieren wird das Auto den Winkel: {alpha_new_my_system} ({math.degrees(alpha_new_my_system)}°) haben. Es hat sich demnach um {delta_alpha} ({math.degrees(delta_alpha)}°) bewegt. Es steht nun an der Position ({x_new}, {y_new}).")

        actions.append(("vorwärts", "links", delta_alpha))
        return rangier_rueckwaerts((x_new, y_new), alpha_new_my_system,
                                   car_rad, p_VA, p_HA, f, w, b, actions)


def rangier_rueckwaerts(car_pos, alpha, car_rad, p_VA, p_HA, f, w, b, actions, verbose = False):
    """
    1. Berechne, wie weit man rückwärts fahren müsste, um vorwärts rauskommen zu können.
    """
    if verbose:
        print(
            f"\nRangiere rückwärts mit car_pos: {car_pos}, alpha: {alpha}({math.degrees(alpha)}°)")

    lower_circle_center = center_of_circle(car_rad, alpha, car_pos, "backward")
    # print(f"Der lower circle center liegt bei: {lower_circle_center}")
    outer_front_car_rad = radius_ecke_vorne_rechts(car_rad, f, w)

    function_to_minimize = dist_func_to_minimize(
        car_rad, outer_front_car_rad, p_VA, alpha, car_pos)
    solutions = fsolve(function_to_minimize, 0)
    # print(f"solutions: {solutions}")
    # print(f"Das Auto müsste {math.degrees(solutions[0])}° nach hinten fahren, damit es vorne gut rauskommt.")

    # todo: Ohne den Wert fährt er gerade so nicht weit genug zurück...
    delta_alpha = solutions[0] + 0.005
    alpha_new = alpha + delta_alpha
    absolute_alpha_new = alpha_new + .5*math.pi

    # print(math.degrees(absolute_alpha_new))
    x_new = lower_circle_center[0] + car_rad * math.cos(absolute_alpha_new)
    y_new = lower_circle_center[1] + car_rad * math.sin(absolute_alpha_new)

    potentielle_hintere_linke_ecke = ecke_hinten_links(
        x_new, y_new, alpha_new, b, w)

    # Man kann so weit zurücksetzen wie gewünscht. Beende die Rekursion
    if potentielle_hintere_linke_ecke[0] > p_HA[0]:
        if verbose:
            print(
                f"Nach dem Rückwärts rangieren wird das Auto den Winkel: {alpha_new} ({math.degrees(alpha_new)}°) haben. Es hat sich demnach um {delta_alpha} ({math.degrees(delta_alpha)}°) bewegt. Es steht nun an der Position: ({x_new}, {y_new})")
        actions.append(("rückwärts", "rechts", delta_alpha))
        return rangier_vorwaerts((x_new, y_new), alpha_new, car_rad, p_VA, p_HA, f, w, b, actions)
    if verbose:
        print(
            f"Leider kann ich nicht so weit zurückfahren, wie ich gerne würde {potentielle_hintere_linke_ecke[1]} < {p_HA[0]}...")
    # Fahre nur soweit zurück, bis das p_HA[0] erreicht wird.
    # TODO: Die Position der potentiellen hinteren linken Ecke validieren!
    outer_back_car_rad = radius_ecke_hinten_links(car_rad, b, w)
    x_hinten_links_new, y_hinten_links_new, alpha_hinten_links_new = pos_und_alpha_hintere_linke_ecke_so_weit_wie_möglich(
        car_rad, outer_back_car_rad, car_pos, alpha, p_HA[0])

    alpha_new = alpha_hinten_links_new + \
        alpha_verschiebung_hinten_links(car_rad, b, w)
    x_new = lower_circle_center[0] + car_rad * math.cos(alpha_new)
    y_new = lower_circle_center[1] + car_rad * math.sin(alpha_new)

    alpha_new_my_system = alpha_new - 0.5 * math.pi
    delta_alpha = alpha_new_my_system - alpha

    if verbose:
        print(
            f"Nach dem Rückwärts SOWEIT WIE MÖGLICH rangieren wird das Auto den Winkel: {alpha_new_my_system} ({math.degrees(alpha_new_my_system)}°) haben. Es hat sich demnach um {delta_alpha} ({math.degrees(delta_alpha)}°) bewegt. Es steht nun an der Position: ({x_new}, {y_new})")
    actions.append(("rückwärts", "rechts", delta_alpha))
    return rangier_vorwaerts((x_new, y_new), alpha_new_my_system,
                             car_rad, p_VA, p_HA, f, w, b, actions)

def pos_und_alpha_hintere_linke_ecke_so_weit_wie_möglich(car_rad, outer_back_car_rad, car_pos, initial_alpha, x_HA):
    """
    Berechnet den x und y Wert und den absoluten Alpha Winkel der hinteren linken Ecke des Autos, wenn es so weit nach hinten fährt, wie es kann (bis es beim x-Wert des hinteren Autos ist.)
    """
    lower_circle_center = center_of_circle(car_rad, initial_alpha, car_pos, "backward")
    alpha = math.acos((x_HA - lower_circle_center[0]) / outer_back_car_rad)
    
    # alpha = 2*math.pi - alpha
    y = lower_circle_center[1] + outer_back_car_rad * math.sin(alpha)
    x = lower_circle_center[0] + outer_back_car_rad * math.cos(alpha)
    # y ist der Ort, an dem die vordere rechte Ecke des Autos landet. Das zugehörige x ist

    return x, y, alpha
    
    
    

def pos_und_alpha_vordere_rechte_ecke_so_weit_wie_möglich(car_rad, outer_front_car_rad, car_pos, initial_alpha, x_VA):
    """ 
    Berechnet, den x und y Wert von der vorderen rechten Ecke des Autos, wenn es so weit nach vorne fährt, wie es kann (bis es beim x-Wert des vorderen Autos ist.)
    Wenn dann der zugehörige y-Wert höher ist, als die obere Beschränkung des Autos, kann zum Ausparken geleitet werden.
    """

    upper_circle_center = center_of_circle(
        car_rad, initial_alpha, car_pos, "forward")

    alpha = math.acos(
        (x_VA - upper_circle_center[0]) / outer_front_car_rad)
    # Das Alpha ist für den oberen Halbkreis, wir brauchen aber das für den unteren:
    alpha = 2*math.pi - alpha
    y = upper_circle_center[1] + outer_front_car_rad * math.sin(alpha)
    x = upper_circle_center[0] + outer_front_car_rad * math.cos(alpha)
    # y ist der Ort, an dem die vordere rechte Ecke des Autos landet. Das zugehörige x ist
    
    return x, y, alpha
    

if __name__ == "__main__":
    """
    Realistisches Beispiel zum Test von delta_y_und_alpha_je_nach_x_nach_vorne
    Auto steht anfangs bei (5.238,10.732)
    Radius des Autos liegt bei: 65.88
    VA_Ecke liegt bei: (50.176, 32.447)
    Das Auto ist um 20° eingelenkt (0.1111111111111111 * pi).
    Das Auto ist von der Hinterachse aus noch 33cm nach vorne und 19 cm breit.

    Erst müsste die Ecke vorne Rechts berechnet werden. In Fusion liegt diese bei: (39.497, 13.092)
    """
    car_pos = (5.238, 10.732)
    target_pos = (50.176, 32.447)
    hinterauto_pos = (5.37, 32.447)
    # hinterauto_pos = (7.844, 32.447)

    initial_alpha = 0.1111111111*math.pi
    car_rad = 65.88
    outer_front_car_rad = 82.2287
    f = 33
    w = 19
    b = 5
    
    x, y, alpha = pos_und_alpha_vordere_rechte_ecke_so_weit_wie_möglich(
        car_rad, outer_front_car_rad, car_pos, initial_alpha, target_pos[0])
    
    """
    x = xm + r * cos a
    a = acos((xm - x)/r)
    y = ym + r * sin a
    Aus der Berechnung von x besorgt man sich das alpha
    und dann kann man das y aus der anderen holen.
    
    PROBLEM: Ich kriege nur die Obere Ecke dessen zu fassen was ich will.
    Aber der gesuchte x-Wert wird ja zwei mal getroffen. Wie sorge ich dafür, dass ich den richtigen finde??
    """
    
    # print(f"Das Alpha nach neuer Rechnung: {alpha} / {math.degrees(alpha)}° resultiert in x:{x}, y: {y}")
    actions = []
    rangier_vorwaerts(car_pos, initial_alpha, car_rad, target_pos, hinterauto_pos, f, w, b, actions)
    """
    Nach dem ersten Rangieren sollte in unserem Beispiel das Auto an der Position 
    (17.121, 16.463) stehen. Das ist korrekt. Außerdem sollte es den Winkel 31.43° haben (also um 11° nach vorne links gefahren sein).
    
    Das Rückwärts rangieren würde berechnen, dass das Auto nun 3.3° rückwärts fahren sollte, um aus der Lücke rauszukommen.
    """
    
    """
    Um das Modell beim Rückwärts fahren richtig zu nerven sollte hinten ein Auto auf Höhe x = 4.5 stehen.
    """
    
    
    """
    Dann wird berechnet, wie weit es noch nach vorne fahren sollte (auf der x-Achse.)
    """
    # x_distanz_nach_vorne = target_pos[0] - x_vr
    
    # dy, dalpha = delta_y_und_alpha_je_nach_x_nach_vorne(x_distanz_nach_vorne, car_rad)
    
    # print(f"Das auto müsste noch {x_distanz_nach_vorne}cm nach vorne (x) fahren. Dazu würde es den Winkel {dalpha} abfahren und dabei {dy} an Höhe gewinnen. Die Endposition wäre also bei: {(car_pos[0]+x_distanz_nach_vorne, car_pos[1]+dy)}")

    
    
    
