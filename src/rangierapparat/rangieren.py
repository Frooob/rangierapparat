import math

from .rangierlib.ablauf import rangier_vorwaerts
from .rangierlib.ausparken import ausparkvorgang

def angle_to_distance(action, car_rad):
    if action[1] in ["rechts", "links"]:
        # Wenn 2 pi = pi * 2 * d sind, dann
        distance = action[2] * car_rad
    else:
        distance = action[2]
        
    return (action[0], action[1], distance)

def translate_action(action):
    front_back = "vorwärts" if action[0] == "rückwärts" else "rückwärts"
    return (front_back, action[1], action[2])

class Rangierer():
    def __init__(self, w=.19, f=.33, b=.05, r=.658, p=.05, l=.45, verbose=False):
        self.verbose = verbose
        self.w = w
        self.f = f
        self.b = b
        self.r = r
        self.p = p
        self.l = l
        # Init:
        self.x_auto = -(l-b)
        self.y_auto = -(p+w)
        self.x_VA = 0
        self.y_VA = -(w/2+p)
        self.x_HA = -l
        self.y_HA = self.y_VA
        
        self.car_pos = (self.x_auto, self.y_auto)
        self.p_VA = (self.x_VA, self.y_VA)
        self.p_HA = (self.x_HA, self.y_HA)
        
        self.actions=[]
        final_x = self.calculate_actions()
        
        self.einpark_actions = self.actions[::-1]
        self.einpark_actions = [angle_to_distance(translate_action(action),self.r) for action in self.einpark_actions]
        self.einpark_actions.insert(0, ("vorwärts", "geradeaus", final_x))
        self.auspark_actions = [angle_to_distance(action, self.r) for action in self.actions]
        
    def calculate_actions(self):
        car_pos_nach_rangieren, alpha_nach_rangieren = rangier_vorwaerts(
            self.car_pos, 0, self.r, self.p_VA, self.p_HA, self.f, self.w, self.b, self.actions, self.verbose)

        # print(
        #     f"car_pos_nach_rangieren: {car_pos_nach_rangieren}, alpha_nach_rangieren: {alpha_nach_rangieren}")
        x_final, y_final = ausparkvorgang(
            car_pos_nach_rangieren, alpha_nach_rangieren, self.r, self.p_VA, self.f, self.w, self.b, 0, self.actions)

        # print(f"x_final: {x_final}")
        return x_final
        
    



if __name__ == "__main__":
    """
    Dieser Test geht einmal den kompletten Algorithmus durch, wie er dann später auch in Wirklichkeit durchgeführt wird. 
    
    Das Auto hat einen Parkplatz erkannt und steht nun mit seiner Hinterachse genau an der Kante, wo das Vorderauto aufhört.
    """
    # hinterauto_pos = (7.844, 32.447)

    car_rad = 65.88
    f = 33
    w = 19
    b = 5
    
    p = 5.306
    l = 49.0
    
    rangierer = Rangierer(w, f, b, car_rad, p, l)
    print("Einparkactions:")
    print(rangierer.einpark_actions)
    print("Ausparkactions:")
    print(rangierer.auspark_actions)
    # actions = []
    
    # # Werte sollten alle mit dem Modell übereinstimmen. 
    # print(f"car_pos: ({rangierer.x_auto}, {rangierer.y_auto})")
    # print(f"p_VA: ({rangierer.x_VA}, {rangierer.y_VA})")
    # print(f"p_HA: ({rangierer.x_HA}, {rangierer.y_HA})")
    
    # car_pos = (rangierer.x_auto, rangierer.y_auto)
    # p_VA = (rangierer.x_VA, rangierer.y_VA)
    # p_HA = (rangierer.x_HA, rangierer.y_HA)
    # car_pos_nach_rangieren, alpha_nach_rangieren = rangier_vorwaerts(car_pos, 0, car_rad, p_VA, p_HA, f, w, b, actions)
    
    # print(f"car_pos_nach_rangieren: {car_pos_nach_rangieren}, alpha_nach_rangieren: {alpha_nach_rangieren}")
    # x_final, y_final = ausparkvorgang(car_pos_nach_rangieren, alpha_nach_rangieren, car_rad, p_VA, f, w, b, 0, actions)
    
    # print(f"x_final: {x_final}")
    # print(actions)
    
