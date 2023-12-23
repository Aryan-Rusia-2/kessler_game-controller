from kesslergame import KesslerController # In Eclipse, the name of the library is kesslergame, not src.kesslergame
from typing import Dict, Tuple
from cmath import sqrt
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import math
from math import sin, acos, degrees
import numpy as np
import matplotlib as plt


class Group10Controller(KesslerController):


    def __init__(self):
        self.eval_frames = 0 #What is this?


    def calc_fire_target(self, a, ship_state, ship_ast_dist):
        
        target_loc = np.array(a['position'])
        ship_loc = np.array(ship_state['position'])
        target_vel = np.array(a['velocity'])

        # calc dist to target
        dist_vec = np.array(ship_loc - target_loc)
        dist = np.linalg.norm(dist_vec)
        # Cacl time for bullet to hit target
        # Bullet speed is 800
        bullet_speed = 800
        time_to_hit = ship_ast_dist/bullet_speed
        # calc loc target will be at when bullet arrives at original target pos
        new_target_loc = target_loc + time_to_hit * target_vel
        # return loc.

        #print('target', target_loc, 'new_target', new_target_loc)
        return new_target_loc

    def has_asteroids_in_path(self, ship_state, turn_rate):
        return abs(turn_rate)<10

    def control_thrust(self, ship_ast_dist, max_thrust, cond):
        
        if (ship_ast_dist < 400) and (cond==True):
      
            thrust = -(max_thrust * (1.0 - min(1.0, ship_ast_dist / max_thrust)))
        else:
            thrust = 0
        return thrust

    def actions(self, ship_state: Dict, game_state: Dict) -> Tuple[float, float, bool]:

        ship_pos_x = ship_state["position"][0]     # See src/kesslergame/ship.py in the KesslerGame Github
        ship_pos_y = ship_state["position"][1]
        closest_asteroid = None

        for a in game_state["asteroids"]:
            #Loop through all asteroids, find minimum Eudlidean distance
            curr_dist = math.sqrt((ship_pos_x - a["position"][0])**2 + (ship_pos_y - a["position"][1])**2)
            if closest_asteroid is None :
                # Does not yet exist, so initialize first asteroid as the minimum. Ugh, how to do?
                closest_asteroid = dict(aster = a, dist = curr_dist)

            else:
                # closest_asteroid exists, and is thus initialized.
                if closest_asteroid["dist"] > curr_dist:
                    # New minimum found
                    closest_asteroid["aster"] = a
                    closest_asteroid["dist"] = curr_dist

        target_loc = self.calc_fire_target(closest_asteroid["aster"], ship_state, closest_asteroid['dist'])
        angle = math.atan2(target_loc[1] - ship_state['position'][1], target_loc[0] - ship_state['position'][0]) * 180/math.pi
        if angle < 0:
            angle += 360

        diff = ship_state['heading'] - angle
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360

        #print("diff",diff)
        turn_rate = -diff*30
        if turn_rate > 180:
            turn_rate =180
        elif turn_rate <-180:
            turn_rate = -180
        
        # print("tr:", turn_rate)
        
        cond = self.has_asteroids_in_path(ship_state, -diff)
        #cond = True
        #print(cond)
        fire_radius = 800
        '''
        if (closest_asteroid["dist"] <= fire_radius) and (cond == True):
            fire = True
        else:
            fire = False
        '''
        fire = True

        # And return your three outputs to the game simulation. Controller algorithm complete.
        thrust = 0.0

        thrust = self.control_thrust(closest_asteroid['dist'], 190 , cond)
        #print("thrust:", thrust)
        self.eval_frames +=1

        #DEBUG
        # print(thrust, bullet_t, shooting_theta, turn_rate, fire)

        if (ship_state['is_respawning'] == True):
            fire = 0
            thrust = self.control_thrust(closest_asteroid['dist'], 160, True)


        return thrust, turn_rate, fire

    @property
    def name(self) -> str:
        return "Group #10"
