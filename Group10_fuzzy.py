from kesslergame import KesslerController # In Eclipse, the name of the library is kesslergame, not src.kesslergame
from typing import Dict, Tuple
from cmath import sqrt
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import math
import numpy as np
import matplotlib as plt

# This is the our controller class. We are using two separate fuzzy systems, one for aiming/firing and another
# one for thrust calculation. 
class Group10fuzzyController(KesslerController):
    
    def __init__(self):
        self.eval_frames = 0 #What is this?

        bullet_time = ctrl.Antecedent(np.arange(0,1.0,0.002), 'bullet_time')
        theta_delta = ctrl.Antecedent(np.arange(-1*math.pi,math.pi,0.1), 'theta_delta') # Radians due to Python
        ship_turn = ctrl.Consequent(np.arange(-180,180,1), 'ship_turn') # Degrees due to Kessler
        ship_fire = ctrl.Consequent(np.arange(-1,1,0.1), 'ship_fire')

        #Declare fuzzy sets for bullet_time (how long it takes for the bullet to reach the intercept point)
        bullet_time['S'] = fuzz.trimf(bullet_time.universe,[0,0,0.05])
        bullet_time['M'] = fuzz.trimf(bullet_time.universe, [0,0.05,0.1])
        bullet_time['L'] = fuzz.smf(bullet_time.universe,0.0,0.1)

        #Declare fuzzy sets for theta_delta 
        theta_delta['NL'] = fuzz.zmf(theta_delta.universe,  -1*math.pi/3,-1*math.pi/6 )
        theta_delta['NS'] = fuzz.trimf(theta_delta.universe, [-0.5235987755982988, -0.2254287924587326, -0.033041334394426414])
        theta_delta['Z'] = fuzz.trimf(theta_delta.universe, [-0.033041334394426414, 0, 0.02375300330926507])
        theta_delta['PS'] = fuzz.trimf(theta_delta.universe,  [0.02375300330926507, 0.34940892236660603, 0.5235987755982988] )
        theta_delta['PL'] = fuzz.smf(theta_delta.universe,math.pi/6,math.pi/3)
      
        #Declare fuzzy sets for ship_turn 
        ship_turn['NL'] = fuzz.trimf(ship_turn.universe, [-180, -122.96823005360848, -81.94270413539232])
        ship_turn['NS'] = fuzz.trimf(ship_turn.universe, [-81.94270413539232, -48.391987681724686, -34.2039567061756])
        ship_turn['Z'] = fuzz.trimf(ship_turn.universe,  [-34.2039567061756, -19.06846995853533, 26.998055174055708])
        ship_turn['PS'] = fuzz.trimf(ship_turn.universe, [26.998055174055708, 38.33165977734129, 86.38604904014232])
        ship_turn['PL'] = fuzz.trimf(ship_turn.universe, [86.38604904014232, 112.85729795889137, 180])
        
        #Declare fuzzy sets for ship_fire 
        ship_fire['N'] = fuzz.trimf(ship_fire.universe, [-1,-1,0.0])
        ship_fire['Y'] = fuzz.trimf(ship_fire.universe, [0.0,1,1])

        #Declare each fuzzy rule
        rule1 = ctrl.Rule(bullet_time['L'] & theta_delta['NL'], (ship_turn['NL'], ship_fire['N']))
        rule2 = ctrl.Rule(bullet_time['L'] & theta_delta['NS'], (ship_turn['NS'], ship_fire['Y']))
        rule3 = ctrl.Rule(bullet_time['L'] & theta_delta['Z'], (ship_turn['Z'], ship_fire['Y']))
        rule4 = ctrl.Rule(bullet_time['L'] & theta_delta['PS'], (ship_turn['PS'], ship_fire['Y']))
        rule5 = ctrl.Rule(bullet_time['L'] & theta_delta['PL'], (ship_turn['PL'], ship_fire['N']))
        rule6 = ctrl.Rule(bullet_time['M'] & theta_delta['NL'], (ship_turn['NL'], ship_fire['N']))
        rule7 = ctrl.Rule(bullet_time['M'] & theta_delta['NS'], (ship_turn['NS'], ship_fire['Y']))
        rule8 = ctrl.Rule(bullet_time['M'] & theta_delta['Z'], (ship_turn['Z'], ship_fire['Y']))
        rule9 = ctrl.Rule(bullet_time['M'] & theta_delta['PS'], (ship_turn['PS'], ship_fire['Y']))
        rule10 = ctrl.Rule(bullet_time['M'] & theta_delta['PL'], (ship_turn['PL'], ship_fire['N']))
        rule11 = ctrl.Rule(bullet_time['S'] & theta_delta['NL'], (ship_turn['NL'], ship_fire['Y']))
        rule12 = ctrl.Rule(bullet_time['S'] & theta_delta['NS'], (ship_turn['NS'], ship_fire['Y']))
        rule13 = ctrl.Rule(bullet_time['S'] & theta_delta['Z'], (ship_turn['Z'], ship_fire['Y']))
        rule14 = ctrl.Rule(bullet_time['S'] & theta_delta['PS'], (ship_turn['PS'], ship_fire['Y']))
        rule15 = ctrl.Rule(bullet_time['S'] & theta_delta['PL'], (ship_turn['PL'], ship_fire['Y']))

        self.targeting_control = ctrl.ControlSystem()
        self.targeting_control.addrule(rule1)
        self.targeting_control.addrule(rule2)
        self.targeting_control.addrule(rule3)
        self.targeting_control.addrule(rule4)
        self.targeting_control.addrule(rule5)
        self.targeting_control.addrule(rule6)
        self.targeting_control.addrule(rule7)
        self.targeting_control.addrule(rule8)
        self.targeting_control.addrule(rule9)
        self.targeting_control.addrule(rule10)
        self.targeting_control.addrule(rule11)
        self.targeting_control.addrule(rule12)
        self.targeting_control.addrule(rule13)
        self.targeting_control.addrule(rule14)
        self.targeting_control.addrule(rule15)

        ## Fuzzy system 2
        ship_ast_dist = ctrl.Antecedent(np.arange(0, 400, 1), 'ship_ast_dist')
        thrust_value = ctrl.Consequent(np.arange(100, 300, 1), 'thrust_value')

        # Define fuzzy membership functions for ship_ast_dist
        ship_ast_dist['close'] = fuzz.trimf(ship_ast_dist.universe, [0, 89.77395389245264, 102.68470071270026])
        ship_ast_dist['medium'] = fuzz.trimf(ship_ast_dist.universe, [102.68470071270026, 203.02309713819037, 331.1079001639199])
        ship_ast_dist['far'] = fuzz.trimf(ship_ast_dist.universe, [331.1079001639199, 372.50478714928306, 400])

        # Define fuzzy membership functions for thrust_value
        thrust_value['low'] = fuzz.trimf(thrust_value.universe, [100, 129.52140784417804, 159.5321242781087])
        thrust_value['medium'] = fuzz.trimf(thrust_value.universe, [159.5321242781087, 170.33418010095988, 207.4593478201578])
        thrust_value['high'] = fuzz.trimf(thrust_value.universe, [207.4593478201578, 292.9486465884094, 300])

        # Define fuzzy rule base
        rule1 = ctrl.Rule(ship_ast_dist['close'], thrust_value['high'])
        rule2 = ctrl.Rule(ship_ast_dist['medium'], thrust_value['medium'])
        rule3 = ctrl.Rule(ship_ast_dist['far'], thrust_value['low'])

        # Create fuzzy control system
        self.targeting_control_for_fuzzy = ctrl.ControlSystem()
        self.targeting_control_for_fuzzy.addrule(rule1)
        self.targeting_control_for_fuzzy.addrule(rule2)
        self.targeting_control_for_fuzzy.addrule(rule3)

    # This function returns true if there are asteroids in the path of the ship. 
    def has_asteroids_in_path(self, turn_rate):
        return abs(turn_rate)<10

    # This function finds the location of intersection of bullet and the asteroid
    def calc_fire_target(self, a, ship_ast_dist):
        target_loc = np.array(a['position'])
        target_vel = np.array(a['velocity'])
        bullet_speed = 800
        time_to_hit = ship_ast_dist/bullet_speed
        new_target_loc = target_loc + time_to_hit * target_vel
        return new_target_loc

    # This function returns the thrust value
    def control_thrust(self, ship_ast_dist, cond):
        if (ship_ast_dist <= 400) & (cond==True):
            thrust_fuzzy = ctrl.ControlSystemSimulation(self.targeting_control_for_fuzzy,flush_after_run=1)
            thrust_fuzzy.input['ship_ast_dist'] = ship_ast_dist
            thrust_fuzzy.compute()
            max_thrust = thrust_fuzzy.output['thrust_value']
            thrust = -(max_thrust * (1.0 - min(1.0, ship_ast_dist / max_thrust)))
        else:
            thrust = 0
        return thrust

    #This is the main function, which returns the ship's decision to the game
    def actions(self, ship_state: Dict, game_state: Dict) -> Tuple[float, float, bool]:
        ship_pos_x = ship_state["position"][0]
        ship_pos_y = ship_state["position"][1]
        closest_asteroid = None

        for a in game_state["asteroids"]:
            curr_dist = math.sqrt((ship_pos_x - a["position"][0])**2 + (ship_pos_y - a["position"][1])**2)
            if closest_asteroid is None :
                closest_asteroid = dict(aster = a, dist = curr_dist)

            else:
                # closest_asteroid exists, and is thus initialized.
                if closest_asteroid["dist"] > curr_dist:
                    # New minimum found
                    closest_asteroid["aster"] = a
                    closest_asteroid["dist"] = curr_dist

        asteroid_ship_x = ship_pos_x - closest_asteroid["aster"]["position"][0]
        asteroid_ship_y = ship_pos_y - closest_asteroid["aster"]["position"][1]

        asteroid_ship_theta = math.atan2(asteroid_ship_y,asteroid_ship_x)

        asteroid_direction = math.atan2(closest_asteroid["aster"]["velocity"][1], closest_asteroid["aster"]["velocity"][0]) # Velocity is a 2-element array [vx,vy].
        my_theta2 = asteroid_ship_theta - asteroid_direction
        cos_my_theta2 = math.cos(my_theta2)
        # Need the speeds of the asteroid and bullet. speed * time is distance to the intercept point
        asteroid_vel = math.sqrt(closest_asteroid["aster"]["velocity"][0]**2 + closest_asteroid["aster"]["velocity"][1]**2)
        bullet_speed = 800 # Hard-coded bullet speed from bullet.py

        # Determinant of the quadratic formula b^2-4ac
        targ_det = (-2 * closest_asteroid["dist"] * asteroid_vel * cos_my_theta2)**2 - (4*(asteroid_vel**2 - bullet_speed**2) * closest_asteroid["dist"])

        # Combine the Law of Cosines with the quadratic formula for solve for intercept time. Remember, there are two values produced.
        intrcpt1 = ((2 * closest_asteroid["dist"] * asteroid_vel * cos_my_theta2) + math.sqrt(targ_det)) / (2 * (asteroid_vel**2 -bullet_speed**2))
        intrcpt2 = ((2 * closest_asteroid["dist"] * asteroid_vel * cos_my_theta2) - math.sqrt(targ_det)) / (2 * (asteroid_vel**2-bullet_speed**2))

        # Take the smaller intercept time, as long as it is positive; if not, take th!pip install scikit-fuzzye larger one.
        if intrcpt1 > intrcpt2:
            if intrcpt2 >= 0:
                bullet_t = intrcpt2
            else:
                bullet_t = intrcpt1
        else:
            if intrcpt1 >= 0:
                bullet_t = intrcpt1
            else:
                bullet_t = intrcpt2

        intrcpt_x = closest_asteroid["aster"]["position"][0] + closest_asteroid["aster"]["velocity"][0] * bullet_t
        intrcpt_y = closest_asteroid["aster"]["position"][1] + closest_asteroid["aster"]["velocity"][1] * bullet_t

        my_theta1 = math.atan2((intrcpt_y - ship_pos_y),(intrcpt_x - ship_pos_x))

        shooting_theta = my_theta1 - ((math.pi/180)*ship_state["heading"])

        shooting_theta = (shooting_theta + math.pi) % (2 * math.pi) - math.pi

        shooting = ctrl.ControlSystemSimulation(self.targeting_control,flush_after_run=1)

        shooting.input['bullet_time'] = bullet_t
        shooting.input['theta_delta'] = shooting_theta

        shooting.compute()

        # Get the defuzzified outputs
        turn_rate = shooting.output['ship_turn']

        if shooting.output['ship_fire'] >= 0:
            fire = True
        else:
            fire = False

        # Finding the target location using the function defined above. 
        target_loc = self.calc_fire_target(closest_asteroid["aster"], closest_asteroid['dist'])
        angle = math.atan2(target_loc[1] - ship_state['position'][1], target_loc[0] - ship_state['position'][0]) * 180/math.pi
        if angle < 0:
            angle += 360

        diff = ship_state['heading'] - angle
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360

        # Checking if there are asteroids in the path of the ship 
        cond = self.has_asteroids_in_path(-diff)

        # Finding the value of thrust using the function describe above
        thrust = self.control_thrust(closest_asteroid['dist'], cond)

        self.eval_frames +=1

        # If the ship is respawning, then don't fire to remain invinsible for 3 seconds
        if (ship_state['is_respawning'] == True):
            fire = 0
            thrust =  -(160 * (1.0 - min(1.0, closest_asteroid['dist'] / 160)))

        return thrust, turn_rate, fire

    @property
    def name(self) -> str:
        return "Group_10"
    

