
from tools.sim.op_script.utils_multiple import append_relevant_paths
if __name__ == "__main__":
    append_relevant_paths()

import socket
import sys
import atexit
import os
import pickle
import argparse
from datetime import datetime
from customized_utils import make_hierarchical_dir, exit_handler
from bridge_multiple_sync3 import run_op_simulation
from multiprocessing import Process, Manager
import traceback

def run_op_simulation_wrapper(x, fuzzing_content, fuzzing_arguments, sim_specific_arguments, dt_arguments, launch_server, counter, port, return_dict):
    objectives, run_info = run_op_simulation(x, fuzzing_content, fuzzing_arguments, sim_specific_arguments, dt_arguments, launch_server, counter, port)
    return_dict['returned_data'] = [objectives, run_info]

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--parent_folder", type=str, default=None)
    parser.add_argument("-m", "--ego_car_model", type=str, default='op')
    parser.add_argument("-m2", "--ego_car_model_final", type=str, default=None)
    parser.add_argument("-w", "--pre_crash_window_length", type=float, default=4)

    arguments = parser.parse_args()
    parent_folder = arguments.parent_folder
    ego_car_model = arguments.ego_car_model
    ego_car_model_final = arguments.ego_car_model_final
    pre_crash_window_length = arguments.pre_crash_window_length

    assert os.path.exists(parent_folder), parent_folder+' does not exist'

    now = datetime.now()
    dt_time_str = now.strftime("%Y_%m_%d_%H_%M_%S")

    sim = None
    subfolder_list = []
    for subfolder in os.listdir(parent_folder):
        cur_folder = os.path.join(parent_folder, subfolder)
        if os.path.isdir(cur_folder):
            subfolder_list.append(subfolder)
    sorted_subfolder = sorted(subfolder_list, key=lambda x:int(x))
    print('sorted_subfolder', sorted_subfolder)
    for i, subfolder in enumerate(sorted_subfolder):
        cur_folder = os.path.join(parent_folder, subfolder)
        cur_info_path = os.path.join(cur_folder, 'cur_info.pickle')
        simulation_data_path = os.path.join(cur_folder, 'simulation_data.pickle')

        if os.path.exists(cur_info_path):
            with open(cur_info_path, 'rb') as f_in_cur_info:
                cur_info = pickle.load(f_in_cur_info)
            with open(simulation_data_path, 'rb') as f_in_simulation_data:
                simulation_data = pickle.load(f_in_simulation_data)

            x = cur_info['x']
            fuzzing_content = cur_info['fuzzing_content']
            fuzzing_arguments = cur_info['fuzzing_arguments']
            sim_specific_arguments = cur_info['sim_specific_arguments']
            dt_arguments = cur_info['dt_arguments']
            if simulation_data['collision'] is not None and 'frame_id' in simulation_data['collision']:
                ego_final_start_time = max([0, simulation_data['collision']['frame_id'] / 100 - pre_crash_window_length])
            else:
                ego_final_start_time = 0


            if 'counter' in cur_info:
                counter = cur_info['counter']
            else:
                counter = int(subfolder)
            print('\n'*3, 'counter', counter, '\n'*3)
            if i == 0:
                launch_server = True
            else:
                launch_server = False
            port = fuzzing_arguments.ports[0]

            atexit.register(exit_handler, fuzzing_arguments.ports)

            fuzzing_arguments.root_folder = 'rerun_op'
            fuzzing_arguments.parent_folder = make_hierarchical_dir([fuzzing_arguments.root_folder, fuzzing_arguments.algorithm_name, fuzzing_arguments.route_type, fuzzing_arguments.scenario_type, fuzzing_arguments.ego_car_model, dt_time_str])
            fuzzing_arguments.mean_objectives_across_generations_path = os.path.join(parent_folder, 'mean_objectives_across_generations.txt')
            fuzzing_arguments.ego_car_model = ego_car_model

            carla_root = os.path.expanduser('~/Documents/self-driving-cars/carla_0911_rss')
            if not os.path.exists(carla_root):
                carla_root = os.path.expanduser('~/Documents/self-driving-cars/carla_0911_no_rss')

            sim_specific_arguments.carla_path = carla_root+"/CarlaUE4.sh"
            if hasattr('sim_specific_arguments', 'ego_car_model_final') and hasattr('sim_specific_arguments', 'ego_final_start_time') and os.path.exists(simulation_data_path):
                sim_specific_arguments.ego_car_model_final = ego_car_model_final
                # This conversion depends on camera frequency and virtual time frequency
                sim_specific_arguments.ego_final_start_time = ego_final_start_time
            else:
                setattr(sim_specific_arguments, 'ego_car_model_final', ego_car_model_final)
                setattr(sim_specific_arguments, 'ego_final_start_time', ego_final_start_time)
            try:
                manager = Manager()
                return_dict = manager.dict()
                objectives, run_info = None, None
                p = Process(target=run_op_simulation_wrapper, args=(x, fuzzing_content, fuzzing_arguments, sim_specific_arguments, dt_arguments, launch_server, counter, port, return_dict))
                p.start()
                p.join(240)
                if p.is_alive():
                    print("Function is hanging!")
                    p.terminate()
                    print("Kidding, just terminated!")
                if 'returned_data' in return_dict:
                    objectives, run_info = return_dict['returned_data']
                else:
                    continue
            except:
                traceback.print_exc()
                continue

            if objectives is not None:
                print('\n'*3)
                print("run_info['is_bug'], run_info['bug_type'], objectives", run_info['is_bug'], run_info['bug_type'], objectives)
                print('\n'*3)
    print('end of program')
    exit(0)
