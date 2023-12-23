# -*- coding: utf-8 -*-
# Copyright © 2022 Thales. All Rights Reserved.
# NOTICE: This file is subject to the license agreement defined in file 'LICENSE', which is part of
# this source code package.

import time

from kesslergame import Scenario, KesslerGame, GraphicsType
from test_controller import TestController
from my_controller import AryanController
# from AryanController import AryanRusiaController
from optimized1_Aryan import RusiaController
from dick_controller import ScottDickController
from graphics_both import GraphicsBoth
from finalController import FinalController
from kesslergame import TrainerEnvironment
from optimized1_Aryan import RusiaController
from kush import  KushController
from hello import Group10Controller
# from aaaa import a
from bestTeam import BestController
from ash_controller7 import AshController7
import pandas as pd
from bestAccuracy import AccuracyController
from god_level import GodController 
from new import BestttController
from final1 import BesttController
# from Group_10 import Group10Controller
from variation1 import VariationoneController
from Aryan_final import a
from Group10_fuzzy import Group10fuzzyController

my_test_scenario = Scenario(name='Test Scenario',
                            num_asteroids=15,
                            ship_states=[
                                {'position': (500, 500), 'angle': 90, 'lives': 3, 'team': 1},
                                {'position': (300, 500), 'angle': 90, 'lives': 3, 'team': 2},

                            ],
                            map_size=(800, 800),
                            time_limit=120,
                            ammo_limit_multiplier=0,
                            stop_if_no_ammo=False)

# Define Game Settings
game_settings = {'perf_tracker': True,
                 'graphics_type': GraphicsType.Tkinter,
                 'realtime_multiplier': 1,
                 'graphics_obj': None,
                 'frequency': 30}

main_dict = {
        'Asteroids hit' : [],
        'Deaths' : [],
        'Accuracy' : [],
        'Evaluated frames' : []
    }
main_dict1 = {
        'Asteroids hit' : [],
        'Deaths' : [],
        'Accuracy' : [],
        'Evaluated frames' : []
    }
a_con = 0
aaaa_con = 0

a_con1 = 0
aaaa_con1 = 0
for i in range(1): 

    game = KesslerGame(settings=game_settings) # Use this to visualize the game scenario
    # game = TrainerEnvironment(settings=game_settings) # Use this for max-speed, no-graphics simulation
    pre = time.perf_counter()
    score, perf_data = game.run(scenario=my_test_scenario, controllers = [AccuracyController(), Group10fuzzyController()])

    main_dict['Asteroids hit'].append([team.asteroids_hit for team in score.teams])
    main_dict['Deaths'].append([team.deaths for team in score.teams])
    main_dict['Accuracy'].append([team.accuracy for team in score.teams])
    main_dict['Evaluated frames'].append([controller.eval_frames for controller in score.final_controllers])
    z = [team.asteroids_hit for team in score.teams]
    a_con += z[0]
    aaaa_con += z[1]


print(a_con, aaaa_con)
df = pd.DataFrame(main_dict)
df.to_csv('output.csv', index=False)

# print(a_con1, aaaa_con1)
# df = pd.DataFrame(main_dict1)
# df.to_csv('output1.csv', index=False)