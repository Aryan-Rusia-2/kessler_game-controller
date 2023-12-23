from kesslergame import KesslerController # In Eclipse, the name of the library is kesslergame, not src.kesslergame
from typing import Dict, Tuple
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import math
import numpy as np
import random
import EasyGA
from kesslergame import Scenario, GraphicsType, TrainerEnvironment


# This is the our controller class. We are using two separate fuzzy systems, one for aiming/firing and another
# one for thrust calculation. 
class Group10FuzzyController(KesslerController):

    def __init__(self, chromosome):
        self.eval_frames = 0 #What is this?

        bullet_time = ctrl.Antecedent(np.arange(0,1.0,0.002), 'bullet_time')
        theta_delta = ctrl.Antecedent(np.arange(-1*math.pi,math.pi,0.1), 'theta_delta') # Radians due to Python
        ship_turn = ctrl.Consequent(np.arange(-180,180,1), 'ship_turn') # Degrees due to Kessler
        ship_fire = ctrl.Consequent(np.arange(-1,1,0.1), 'ship_fire')

        #Declare fuzzy sets for bullet_time (how long it takes for the bullet to reach the intercept point)
        bullet_time['S'] = fuzz.trimf(bullet_time.universe,[0,0,0.05])
        bullet_time['M'] = fuzz.trimf(bullet_time.universe, [0,0.05,0.1])
        bullet_time['L'] = fuzz.smf(bullet_time.universe,0.0,0.1)

        theta_delta['NL'] = fuzz.zmf(theta_delta.universe,  -1*math.pi/3,-1*math.pi/6 )
        theta_delta['NS'] = fuzz.trimf(theta_delta.universe, chromosome['NS'])
        theta_delta['Z'] = fuzz.trimf(theta_delta.universe, chromosome['Z'])
        theta_delta['PS'] = fuzz.trimf(theta_delta.universe,  chromosome['PS'] )
        theta_delta['PL'] = fuzz.smf(theta_delta.universe,math.pi/6,math.pi/3)

        ship_turn['NL'] = fuzz.trimf(ship_turn.universe, chromosome['ship_NL'])
        ship_turn['NS'] = fuzz.trimf(ship_turn.universe, chromosome['ship_NS'])
        ship_turn['Z'] = fuzz.trimf(ship_turn.universe,  chromosome['ship_Z'])
        ship_turn['PS'] = fuzz.trimf(ship_turn.universe, chromosome['ship_PS'])
        ship_turn['PL'] = fuzz.trimf(ship_turn.universe, chromosome['ship_PL'])

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

        ## Fuzzy system 2 for finding the thrust values
        ship_ast_dist = ctrl.Antecedent(np.arange(0, 400, 1), 'ship_ast_dist')
        thrust_value = ctrl.Consequent(np.arange(150, 250, 1), 'thrust_value')

        # Define fuzzy membership functions for ship_ast_dist
        ship_ast_dist['close'] = fuzz.trimf(ship_ast_dist.universe,  chromosome['close'])
        ship_ast_dist['medium'] = fuzz.trimf(ship_ast_dist.universe, chromosome['medium'])
        ship_ast_dist['far'] = fuzz.trimf(ship_ast_dist.universe, chromosome['far'])

        # Define fuzzy membership functions for thrust_value
        thrust_value['low'] = fuzz.trimf(thrust_value.universe, chromosome['low'])
        thrust_value['medium'] = fuzz.trimf(thrust_value.universe, chromosome['mid'])
        thrust_value['high'] = fuzz.trimf(thrust_value.universe, chromosome['high'])

        # Define fuzzy rule base
        rule_1 = ctrl.Rule(ship_ast_dist['close'], thrust_value['high'])
        rule_2 = ctrl.Rule(ship_ast_dist['medium'], thrust_value['medium'])
        rule_3 = ctrl.Rule(ship_ast_dist['far'], thrust_value['low'])

        # Create fuzzy control system
        self.targeting_control_for_fuzzy = ctrl.ControlSystem()
        self.targeting_control_for_fuzzy.addrule(rule_1)
        self.targeting_control_for_fuzzy.addrule(rule_2)
        self.targeting_control_for_fuzzy.addrule(rule_3)

    # This function returns true if there are asteroids in the path of the ship. 
    def has_asteroids_in_path(self, turn_rate):
        return abs(turn_rate)<10

    # This function finds the location of intersection of bullet and the asteroid
    def calc_fire_target(self, a, ship_ast_dist):
        target_loc = np.array(a['position'])
        target_vel = np.array(a['velocity'])
        bullet_speed = 800
        time_to_hit = ship_ast_dist/bullet_speed
        new_target_loc = target_loc + (time_to_hit * target_vel)
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

        cond = self.has_asteroids_in_path(-diff)

        # Finding the value of thrust using the function describe above
        thrust = self.control_thrust(closest_asteroid['dist'], cond)

        self.eval_frames +=1

        # If the ship is respawning, then don't fire to remain invinsible for 3 seconds
        if (ship_state['is_respawning'] == True):
            fire = 0
            thrust = self.control_thrust(closest_asteroid['dist'], True)

        # Return the values of thrust, turn_rate and fire
        return thrust, turn_rate, fire

    @property
    def name(self) -> str:
        return "Group10FuzzyController"


# This function generates the chromosomes required in the EasyGA genetic algorithm
def generate_chromosome():

    thet_values = sorted([random.uniform(-1*math.pi/6, 0) for _ in range(2)])
    ns_mid, ns_end = thet_values

    thet_values = sorted([random.uniform(0, math.pi/6) for _ in range(2)])
    z_end , ps_mid = thet_values

    ship_values = sorted([random.uniform(-180, 180) for _ in range(9)])
    ship_nl_mid, ship_nl_end, ship_ns_mid, ship_ns_end, ship_z_mid, ship_z_end , ship_ps_mid , ship_ps_end , ship_pl_mid  = ship_values

    distance = sorted([random.uniform(0, 400) for _ in range(5)])
    close_mid , close_high, medium_mid, medium_high , far_mid = distance

    thrust_max = sorted([random.uniform(150, 250) for _ in range(5)])
    low_mid , low_high, mid_mid ,mid_high , high_mid   = thrust_max

    chromosome = {

        # These are for the theta values
        "NS" : [-1*math.pi/6, ns_mid, ns_end],
        "Z" : [ns_end, 0, z_end],
        "PS" : [z_end, ps_mid,  math.pi/6],

        # These are for the theta values of ship
        "ship_NL" : [-180 , ship_nl_mid, ship_nl_end],
        "ship_NS" : [ship_nl_end, ship_ns_mid, ship_ns_end],
        "ship_Z" : [ship_ns_end, ship_z_mid, ship_z_end],
        "ship_PS" : [ship_z_end, ship_ps_mid, ship_ps_end],
        "ship_PL" : [ship_ps_end, ship_pl_mid, 180],

        # These are for the distance of closest asteroid from the ship
        "close" : [0 , close_mid, close_high],
        "medium" : [close_high, medium_mid, medium_high],
        "far" : [medium_high, far_mid, 400],

        # These are for the thrust values outputted by the 2nd fuzzy system
        "low" : [150, low_mid, low_high],
        "mid" : [low_high, mid_mid, mid_high],
        "high" : [mid_high, high_mid, 250],
    }

    return chromosome


# The most important function in genetic algorithm, the "Fitness" function.
# It contains the complete test scenario for the kessler game, and passes the generated
# chromosomes to our controller for achieving the best params. 
def fitness(chromosome):
   for i in chromosome:
       chromosomes = i.value

   # Describes the test scenario for the kessler game
   my_test_scenario = Scenario(name='Test Scenario',
                           num_asteroids=10,
                           ship_states=[
                               {'position': (400, 400), 'angle': 90, 'lives': 3, 'team': 1, }],
                           map_size=(1000, 800),
                           time_limit=60,
                           ammo_limit_multiplier=0,
                           stop_if_no_ammo=False)


   # Define Game Settings
   game_settings = {'perf_tracker': True,
                'graphics_type': GraphicsType.Tkinter,
                'realtime_multiplier': 1,
                'graphics_obj': None,
                'frequency': 30}

   my_controller = 0

   for i in range(2):
      game = TrainerEnvironment(settings=game_settings) # Use this for max-speed, no-graphics simulation
      score, perf_data = game.run(scenario=my_test_scenario, controllers = [Group10FuzzyController(chromosomes)])
      my_controller += [team.asteroids_hit for team in score.teams][0]
      
   print(my_controller)
   return my_controller

# This fucntion contains all the code for EasyGA, and it calls fitness as well as the generate 
# chromosme function. 
def Easy_Ga():
   ga = EasyGA.GA()
   ga.gene_impl = lambda: generate_chromosome()
   ga.chromosome_length = 1
   ga.population_size = 6
   ga.target_fitness_type = 'max'
   ga.generation_goal = 1
   ga.fitness_function_impl = fitness
   ga.evolve()
   ga.print_best_chromosome()

# Finally calls the EasyGA function to get the best parameters
Easy_Ga()