import numpy as np
from tools.sim.op_script.object_params import Vehicle, VehicleBehavior
from tools.sim.op_script.scene_configs import customized_routes
from tools.sim.op_script.setup_labels_and_bounds import emptyobject
import os
import shlex
import traceback
import logging
import subprocess
import socket
import time
import math
import carla
import pickle
from sklearn.neighbors import KDTree

# TBD: import from customized_utils by fixing dependency issue
# copy from customized_utils
def is_distinct_vectorized(cur_X, prev_X, mask, xl, xu, p, c, th, verbose=True):
    if len(cur_X) == 0:
        return []
    cur_X = np.array(cur_X)
    prev_X = np.array(prev_X)
    eps = 1e-10
    remaining_inds = np.arange(cur_X.shape[0])

    mask = np.array(mask)
    xl = np.array(xl)
    xu = np.array(xu)

    n = len(mask)

    variant_fields = (xu - xl) > eps
    variant_fields_num = np.sum(variant_fields)
    th_num = np.max([np.round(th * variant_fields_num), 1])

    mask = mask[variant_fields]
    int_inds = mask == "int"
    real_inds = mask == "real"
    xl = xl[variant_fields]
    xu = xu[variant_fields]
    xl = np.concatenate([np.zeros(np.sum(int_inds)), xl[real_inds]])
    xu = np.concatenate([0.99*np.ones(np.sum(int_inds)), xu[real_inds]])

    # hack: backward compatibility with previous run data
    # if cur_X.shape[1] == n-1:
    #     cur_X = np.concatenate([cur_X, np.zeros((cur_X.shape[0], 1))], axis=1)

    cur_X = cur_X[:, variant_fields]
    cur_X = np.concatenate([cur_X[:, int_inds], cur_X[:, real_inds]], axis=1) / (np.abs(xu - xl) + eps)

    if len(prev_X) > 0:
        prev_X = prev_X[:, variant_fields]
        prev_X = np.concatenate([prev_X[:, int_inds], prev_X[:, real_inds]], axis=1) / (np.abs(xu - xl) + eps)

        diff_raw = np.abs(np.expand_dims(cur_X, axis=1) - np.expand_dims(prev_X, axis=0))
        diff = np.ones(diff_raw.shape) * (diff_raw > c)
        diff_norm = np.linalg.norm(diff, p, axis=2)
        equal = diff_norm < th_num
        remaining_inds = np.mean(equal, axis=1) == 0
        remaining_inds = np.arange(cur_X.shape[0])[remaining_inds]

        # print('remaining_inds', remaining_inds, np.arange(cur_X.shape[0])[remaining_inds], cur_X[np.arange(cur_X.shape[0])[remaining_inds]])
        if verbose:
            print('prev X filtering:',cur_X.shape[0], '->', len(remaining_inds))

    if len(remaining_inds) == 0:
        return []

    cur_X_remaining = cur_X[remaining_inds]
    print('len(cur_X_remaining)', len(cur_X_remaining))
    unique_inds = []
    for i in range(len(cur_X_remaining)-1):
        diff_raw = np.abs(np.expand_dims(cur_X_remaining[i], axis=0) - cur_X_remaining[i+1:])
        diff = np.ones(diff_raw.shape) * (diff_raw > c)
        diff_norm = np.linalg.norm(diff, p, axis=1)
        equal = diff_norm < th_num
        if np.mean(equal) == 0:
            unique_inds.append(i)

    unique_inds.append(len(cur_X_remaining)-1)

    if verbose:
        print('cur X filtering:',cur_X_remaining.shape[0], '->', len(unique_inds))

    if len(unique_inds) == 0:
        return []
    remaining_inds = remaining_inds[np.array(unique_inds)]


    return remaining_inds



def convert_x_to_customized_data(
    x,
    fuzzing_content,
    port
):

    num_of_vehicle_behavior_changes = fuzzing_content.search_space_info.num_of_vehicle_behavior_changes
    num_of_vehicles_max = fuzzing_content.search_space_info.num_of_vehicles_max
    # customized_center_transforms = fuzzing_content.customized_center_transforms


    # parameters
    # global
    weather = x[:10].tolist()
    num_of_vehicles = int(x[10])
    delay_time_to_start = int(x[11])
    ego_maximum_speed = int(x[12])
    ind = 13

    # vehicles
    vehicles_list = []
    for i in range(num_of_vehicles_max):
        if i < num_of_vehicles:
            vehicle_type_i = int(x[ind])
            vehicle_x_i = x[ind+1]
            vehicle_y_i = x[ind+2]
            vehicle_yaw_i = x[ind+3]
            vehicle_speed_i = x[ind+4]
            ind += 5

            vehicle_behaviors_i = []
            for _ in range(num_of_vehicle_behavior_changes):
                vehicle_behaviors_i.append(VehicleBehavior(x[ind], x[ind+1]))
                ind += 2

            vehicle_i = Vehicle(
                model=vehicle_type_i,
                x=vehicle_x_i,
                y=vehicle_y_i,
                yaw=vehicle_yaw_i,
                speed=vehicle_speed_i,
                behaviors=vehicle_behaviors_i,
            )

            vehicles_list.append(vehicle_i)

        else:
            ind += 5 + num_of_vehicle_behavior_changes * 2

    customized_data = {
        "weather": weather,
        "delay_time_to_start": delay_time_to_start,
        "vehicles_list": vehicles_list,
        "num_of_vehicle_behavior_changes": num_of_vehicle_behavior_changes,
        "ego_maximum_speed": ego_maximum_speed,
        "customized_center_transforms": fuzzing_content.customized_center_transforms,
    }

    return customized_data

def initialize_op_specific(fuzzing_arguments):

    route_info = customized_routes[fuzzing_arguments.route_type]
    x, y, _, _, yaw, _ = route_info['location_list'][0]
    ego_start_position = (x, y, yaw)

    sim_specific_arguments = emptyobject(
                                route_info=route_info, ego_start_position=ego_start_position, carla_path=fuzzing_arguments.carla_path, client=None,
                                ego_car_model_final=None,
                                ego_final_start_time=0,
                                kd_tree=None)


    return sim_specific_arguments

def get_lead_interseced_keys(simulation_data):
    camera_leads_dict_keys = set(list(simulation_data['camera_leads_dict'].keys()))
    radar_clusters_dict_keys = set(list(simulation_data['radar_clusters_dict'].keys()))
    lead_d_carla_dict_keys = set(list(simulation_data['lead_d_carla_dict'].keys()))
    leads_predicted_list_dict_keys = set(list(simulation_data['leads_predicted_list_dict'].keys()))

    keys_intersected = camera_leads_dict_keys & radar_clusters_dict_keys & lead_d_carla_dict_keys & leads_predicted_list_dict_keys

    keys_union = camera_leads_dict_keys | radar_clusters_dict_keys | lead_d_carla_dict_keys | leads_predicted_list_dict_keys

    keys_intersected_sorted = sorted(keys_intersected)
    return keys_intersected_sorted

def record_lead_and_error(simulation_data, simulation_data_formated_path, keys_intersected_sorted, last_n_frame_id):
    total_count = 0
    error_count = 0
    fusion_error_both_count = 0
    fusion_error_cam_count = 0
    fusion_error_radar_count = 0
    with open(simulation_data_formated_path, 'w') as f_out:
        end_frame_id = simulation_data['end_frame_id']
        if end_frame_id == 0:
            end_frame_id = keys_intersected_sorted[-1]
        # print('keys_intersected_sorted[-1], end_frame_id - last_n_frame_id', keys_intersected_sorted[-1], end_frame_id - last_n_frame_id, len(keys_intersected_sorted))
        for k in keys_intersected_sorted:
            cam_k = simulation_data['camera_leads_dict'][k]
            radar_k = simulation_data['radar_clusters_dict'][k]
            # backward compatibility
            if 'radar_clusters_raw_dict' in simulation_data:
                radar_raw_k = simulation_data['radar_clusters_raw_dict'][k]
            carla_k = simulation_data['lead_d_carla_dict'][k]
            pred_k = simulation_data['leads_predicted_list_dict'][k]

            # hack: for collision error only consider the case when there is a leading vechile and the prediction is wrong but not taking into account the other way
            if carla_k['status'] == True:
                cam_total_error = measure_error(cam_k, carla_k)
                radar_total_error = measure_error(radar_k, carla_k)
                pred_total_error = measure_error(pred_k, carla_k)
            else:
                cam_total_error = radar_total_error = pred_total_error = 0

            if k >= end_frame_id - last_n_frame_id:
                if pred_total_error > 0:
                    error_count += 1
                    if cam_total_error <= 0 and radar_total_error <= 0:
                        fusion_error_both_count += 1
                    elif cam_total_error <= 0:
                        fusion_error_cam_count += 1
                    elif radar_total_error <= 0:
                        fusion_error_radar_count += 1
                total_count += 1

            f_out.write('frame_id: '+str(k)+'\n')
            f_out.write('camera_leads_dict: '+str(cam_k)+'\n'*2)
            f_out.write('radar_clusters_dict: '+str(radar_k)+'\n'*2)
            # backward compatibility
            if 'radar_clusters_raw_dict' in simulation_data:
                f_out.write('radar_clusters_raw_dict: '+str(radar_raw_k)+'\n'*2)
            f_out.write('lead_d_carla_dict: '+str(carla_k)+'\n'*2)
            f_out.write('leads_predicted_list_dict: '+str(pred_k)+'\n'*2)
            f_out.write('cam_total_error: '+str(cam_total_error)+'\n')
            f_out.write('radar_total_error: '+ str(radar_total_error)+'\n')
            f_out.write('pred_total_error: '+str(pred_total_error)+'\n')
            f_out.write('total_count, error_count, fusion_error_both_count, fusion_error_cam_count, fusion_error_radar_count: '+','.join([str(s) for s in [total_count, error_count, fusion_error_both_count, fusion_error_cam_count, fusion_error_radar_count]])+'\n'*2)
            if k in simulation_data['d_avg_list_dict']:
                f_out.write('d_avg_list_dict: '+str(simulation_data['d_avg_list_dict'][k])+'\n')

            # backward compatibility
            if 'd_avg_old_list_dict' in simulation_data and k in simulation_data['d_avg_old_list_dict']:
                f_out.write('d_avg_old_list_dict: '+str(simulation_data['d_avg_old_list_dict'][k])+'\n')

            if k in simulation_data['vehicle_info_dict']:
                f_out.write('vehicle_info_dict: '+str(simulation_data['vehicle_info_dict'][k])+'\n')

            f_out.write('-'*30+'\n'*3)


    return total_count, error_count, fusion_error_both_count, fusion_error_cam_count, fusion_error_radar_count


# def record_pure_vehicle_info(simulation_data, simulation_data_vehicle_info_path):
#     with open(simulation_data_vehicle_info_path, 'w') as f_out:
#         for k, vehicle_info_k in simulation_data['vehicle_info_dict'].items():
#             f_out.write('vehicle_info_k: '+str(vehicle_info_k)+'\n'*2)

def estimate_trajectory_vector(kd_tree, simulation_data_path, max_speed_in_km_per_h, speed_grid_num):

    max_speed_in_m_per_s = max_speed_in_km_per_h / (1.61*2.237)
    speed_grid_step = max_speed_in_m_per_s / speed_grid_num

    n_nodes = len(kd_tree.data)
    yv_region_cov = np.zeros([n_nodes, speed_grid_num])

    with open(simulation_data_path, 'rb') as f_in:
        simulation_data = pickle.load(f_in)

    speed_dict = {}
    for x, y, speed in simulation_data['trajectory_dict'].values():
        dists, inds = kd_tree.query(np.array([[x, y]]), k=1)
        dist = dists[0][0]
        ind = inds[0][0]
        if ind not in speed_dict:
            speed_dict[ind] = [speed]
        else:
            speed_dict[ind].append(speed)
        # print('x, y, speed, ind', x, y, speed, ind)


    for ind, speed in speed_dict.items():
        cur_speed = np.mean(speed_dict[ind])
        speed_ind = np.min([int(cur_speed // speed_grid_step), speed_grid_num-1])
        yv_region_cov[ind, speed_ind] = 1
        # print('ind, cur_speed, speed_ind', ind, cur_speed, speed_ind)

    print('yv_region_cov', yv_region_cov)
    return yv_region_cov.flatten()


def estimate_objectives(simulation_data_path, temporary_running_folder, last_n_frame_id):
    min_d = 130
    collision = 0
    speed = 0
    d_angle_norm = 1
    is_bug = 0
    fusion_error_perc = 0

    simulation_data = None

    # t_0 = time.time()
    success = False
    # while time.time() - t_0 < 10:

    if os.path.exists(simulation_data_path) and os.path.getsize(simulation_data_path) > 0:
        try:
            with open(simulation_data_path, 'rb') as f_in:
                simulation_data = pickle.load(f_in)
                success = True
            # break
        except:
            print('simulation_data_path', simulation_data_path, 'is not ready')
            traceback.print_exc()

    if success:
        if simulation_data['end_condition'] == 'collision':
            collision = 1
            speed = simulation_data['collision']['speed']
            d_angle_norm = simulation_data['collision']['d_angle_norm']

        if simulation_data['d_avg_list_dict']:
            min_d = np.min(np.array(list(simulation_data['d_avg_list_dict'].values())))

        keys_intersected_sorted = get_lead_interseced_keys(simulation_data)

        simulation_data_formated_path = os.path.join(temporary_running_folder, 'simulation_data_formated.txt')

        simulation_data_vehicle_info_path = os.path.join(temporary_running_folder, 'simulation_data_vehicle_info.txt')

        total_count, error_count, fusion_error_both_count, fusion_error_cam_count, fusion_error_radar_count = record_lead_and_error(simulation_data, simulation_data_formated_path, keys_intersected_sorted, last_n_frame_id)

        # record_pure_vehicle_info(simulation_data, simulation_data_vehicle_info_path)

        if total_count > 0:
            fusion_error_perc = (fusion_error_both_count+fusion_error_cam_count+fusion_error_radar_count) / total_count

        tmp_objectives = np.array([min_d, collision, speed, d_angle_norm])
        is_bug = check_bug(tmp_objectives)
    else:
        print('fail to read the pickle file at:', simulation_data_path)

    # normalize min_d
    min_d_normalized = np.min([min_d/130, 1])
    objectives = np.array([min_d_normalized, collision, speed, d_angle_norm, is_bug, fusion_error_perc, 0.])

    return objectives

def check_bug(objectives):
    if objectives[1] == 1 and np.abs(objectives[2]) > 0.1 and objectives[3] < 1e-5:
        return 1
    else:
        return 0



# TBD: try to define a quantitative creteria, right now is by hand
def classify_bug_type(objectives, object_type=''):
    return 1, ''

# TBD: currently consider all bugs as unique bugs
def get_unique_bugs(
    X, objectives_list, mask, xl, xu, unique_coeff, objective_weights, return_mode='unique_inds_and_interested_and_bugcounts', consider_interested_bugs=1, bugs_type_list=[], bugs=[], bugs_inds_list=[], trajectory_vector_list=[]
):

    if len(bugs) == 0:
        for i, (x, objectives) in enumerate(zip(X, objectives_list)):
            if check_bug(objectives):
                bug_type, _ = classify_bug_type(objectives)
                bugs_type_list.append(bug_type)
                bugs.append(x)
                bugs_inds_list.append(i)

    if len(trajectory_vector_list) > 0 and trajectory_vector_list[0] is not None and len(bugs_inds_list) > 0:
        print('bugs_inds_list', bugs_inds_list)
        print('np.array(bugs_inds_list)', np.array(bugs_inds_list))

        chosen_objectives_np = np.array(objectives_list)[np.array(bugs_inds_list)]

        # only consider highly-likely fusion-induced errors when considering unique_bugs for counting and sampling
        remaining_inds = []
        for i, obj in enumerate(chosen_objectives_np):
            if obj[5] > 0.05:
                remaining_inds.append(i)



        # chosen_trajectory_vector_np = np.array(trajectory_vector_list)[np.array(bugs_inds_list)]
        # n_cov = chosen_trajectory_vector_np.shape[1]
        #
        # mask_in_effect = ['float']*n_cov
        # xl_in_effect = [0]*n_cov
        # xu_in_effect = [1]*n_cov
        # p = 0
        # c = 0.5
        # # hack: need to be customizable at interface as that in trajectory_analysis - plot_trajectory_heatmap
        # th = 1 / n_cov
        #
        # remaining_inds = is_distinct_vectorized(chosen_trajectory_vector_np, [], mask_in_effect, xl_in_effect, xu_in_effect, p, c, th, verbose=True)


        print('bugs', bugs)
        print('remaining_inds', remaining_inds)
        unique_bugs = (np.array(bugs)[remaining_inds]).tolist()
        print('len(bugs), len(unique_bugs)', len(bugs), len(unique_bugs))

        unique_bugs_inds_list = (np.array(bugs_inds_list)[remaining_inds]).tolist()
    else:
        unique_bugs = bugs
        unique_bugs_inds_list = bugs_inds_list

    interested_unique_bugs = unique_bugs

    num_of_collisions = len(bugs)
    unique_collision_num = len(unique_bugs)

    if return_mode == 'unique_inds_and_interested_and_bugcounts':
        return unique_bugs, unique_bugs_inds_list, interested_unique_bugs, [num_of_collisions,
        unique_collision_num]
    elif return_mode == 'return_bug_info':
        return unique_bugs, (bugs, bugs_type_list, bugs_inds_list, interested_unique_bugs)

# TBD: currently activate both
def choose_weight_inds(objective_weights):
    weight_inds = np.arange(0, len(objective_weights))
    return weight_inds


def determine_y_upon_weights(objective_list, objective_weights):
    # TBD: consider all bugs for now
    y = np.zeros(len(objective_list))
    for i, obj in enumerate(objective_list):
        y[i] = check_bug(obj)
    return y

def get_all_y(objective_list, objective_weights):
    # TBD: consider all bugs for now
    y_list = np.zeros((1, len(objective_list)))
    for i, obj in enumerate(objective_list):
        y_list[0, i] = check_bug(obj)
    return y_list

def is_port_in_use(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex(("localhost", int(port))) == 0

def start_server(port, carla_path):
    # ./CarlaUE4.sh -carla-rpc-port=2003 -carla-streaming-port=0 -nosound -quality-level=Epic -opengl
    cmd_list = shlex.split(
        "sh "+carla_path+" -carla-rpc-port="
        + str(port)
        + " -carla-streaming-port=0 -nosound -quality-level=Epic -opengl"
    )
    while is_port_in_use(int(port)):
        try:
            subprocess.run("kill $(lsof -t -i:" + str(port) + ")", shell=True)
            # subprocess.run("pkill -f 'CarlaUE4-Linux-'", shell=True)

            print("-" * 20, "kill server at port", port)
            time.sleep(2)
        except:
            traceback.print_exc()
            continue
    subprocess.Popen(cmd_list)
    print("-" * 20, "start server at port", port)
    # 10s is usually enough
    time.sleep(15)

def start_client(host, port):
    print('initialize carla client')
    client = None
    while True:
        try:
            client = carla.Client(host, port)
            client.set_timeout(20.0)
            break
        except:
            print('start client error')
            traceback.print_exc()
    return client


def try_load_world(town_name, client, host, port, carla_path):

    while True:
        try:
            client.set_timeout(20.0)
            world = client.load_world(town_name)
            break
        except:
            traceback.print_exc()
            print('restart server')
            start_server(port, carla_path)
            print('restart client')
            client = carla.Client(host, port)
    return world, client

def create_transform(x, y, z, pitch, yaw, roll):
    location = carla.Location(x, y, z)
    rotation = carla.Rotation(pitch, yaw, roll)
    transform = carla.Transform(location, rotation)
    return transform


def angle_from_center_view_fov(target_location, ego, fov=70):
    ego_location = ego.get_location()
    ego_forward = ego.get_transform().rotation.get_forward_vector()

    # adjust for camera location
    ego_location = ego.get_location() #+ 0.8 * ego_forward

    target_vector = target_location - ego_location

    target_vector = np.array([target_vector.x, target_vector.y])
    ego_forward = np.array([ego_forward.x, ego_forward.y])

    norm_target = np.linalg.norm(target_vector)
    target_vector /= norm_target

    if norm_target < 0.001:
        return 0, (ego_forward[0], ego_forward[1], target_vector[0], target_vector[1])

    try:
        d_angle = np.abs(
            math.degrees(math.acos(np.dot(ego_forward, target_vector)))
        )
    except Exception as ex:
        print(
            "\n" * 3,
            "np.dot(forward_vector, target_vector)",
            np.dot(ego_forward, target_vector),
            "\n" * 3,
        )
        print('type(ex).__name__', type(ex).__name__)
        d_angle = 0
    # d_angle_norm == 0 when target within fov
    d_angle_norm = np.clip((d_angle - fov / 2) / (180 - fov / 2), 0, 1)

    return d_angle_norm, (ego_forward[0], ego_forward[1], target_vector[0], target_vector[1])


def create_transform(transform_values):
    position = carla.Transform(carla.Location(x=transform_values[0], y=transform_values[1], z=transform_values[2]), carla.Rotation(pitch=transform_values[3], yaw=transform_values[4], roll=transform_values[5]))
    return position

def measure_error(tested_leads, ground_truth_lead):
    dRel_th = 4
    yRel_th = 1
    vRel_th = 2.5
    total_error = 3
    for tested_lead in tested_leads:
        dRel_error, yRel_error, vRel_error = measure_error_between_two_leads(tested_lead, ground_truth_lead)

        dRel_error_normalized = np.clip((np.abs(dRel_error) - dRel_th) / dRel_th, 0, 1)
        yRel_error_normalized = np.clip((np.abs(yRel_error) - yRel_th) / yRel_th, 0, 1)
        vRel_error_normalized = np.clip((np.abs(vRel_error) - vRel_th) / vRel_th, 0, 1)

        current_total_error = dRel_error_normalized + yRel_error_normalized + vRel_error_normalized

        total_error = np.min([total_error, current_total_error])

    return total_error

def measure_error_between_two_leads(tested_lead, ground_truth_lead):
    if tested_lead["modelProb"] >= 0.5:
        tested_lead_dRel = tested_lead["dRel"]
        tested_lead_yRel = tested_lead["yRel"]
        tested_lead_vRel = tested_lead["vRel"]
    else:
        tested_lead_dRel = 0
        tested_lead_yRel = 0
        tested_lead_vRel = 0
    # print('ground_truth_lead', ground_truth_lead)
    dRel_error = tested_lead_dRel - ground_truth_lead["dRel"]
    yRel_error = tested_lead_yRel - ground_truth_lead["yRel"]
    vRel_error = tested_lead_vRel - ground_truth_lead["vRel"]

    return dRel_error, yRel_error, vRel_error

def get_inds_with_errors(tested_leads, ground_truth_lead, type):
    dRel_th = 4
    yRel_th = 1
    vRel_th = 2

    error_type_ind_list = []
    for ind, tested_lead in enumerate(tested_leads):
        if tested_lead['modelProb'] >= 0.5:
            dRel_error, yRel_error, vRel_error = measure_error_between_two_leads(tested_lead, ground_truth_lead)
            current_total_error = np.abs(dRel_error)/dRel_th + np.abs(yRel_error)/yRel_th + np.abs(vRel_error)/vRel_th

            error_type_ind_list.append((current_total_error, type, ind))

    return error_type_ind_list


def convert_lead_carla_to_dict(lead_carla):
    lead_carla_dict = {
        "dRel": lead_carla.dRel,
        "yRel": lead_carla.yRel,
        "vRel": lead_carla.vRel,
        "aLeadK": lead_carla.aLeadK,
        "status": lead_carla.status,
        "modelProb": lead_carla.modelProb,
    }
    return lead_carla_dict


def get_job_results(tmp_run_info_list, x_sublist, objectives_sublist_non_traj, trajectory_vector_sublist, x_list, objectives_list, trajectory_vector_list, traj_dist_metric='nearest'):

    def get_fusion_error_proxy_inds(objectives_list_no_traj):
        fusion_error_proxy_inds = []
        for ind, obj in enumerate(objectives_list_no_traj):
            if obj[4] == 1 and obj[5] > 0.05:
                fusion_error_proxy_inds.append(ind)
        return fusion_error_proxy_inds


    print('traj_dist_metric', traj_dist_metric)
    traj_dist_metric = 'conditional_nearest'
    assert traj_dist_metric in ['nearest', 'average', 'conditional_nearest', 'conditional_average']

    route_interval_num = 1
    for i in range(len(tmp_run_info_list)):
        if tmp_run_info_list[i] is not None and 'sim_specific_arguments' in tmp_run_info_list[i]:
            route_interval_num = len(tmp_run_info_list[0]['sim_specific_arguments'].kd_tree.data)
            break
    print('route_interval_num', route_interval_num)


    # update all run data
    # print('objectives_list', objectives_list)
    if len(x_list) > 0:
        objectives_list_no_traj = objectives_list + objectives_sublist_non_traj
        trajectory_vector_list = trajectory_vector_list + trajectory_vector_sublist
        x_list = x_list + x_sublist
    else:
        objectives_list_no_traj = objectives_sublist_non_traj
        trajectory_vector_list = trajectory_vector_sublist
        x_list = x_sublist


    # update diversity density value
    # replace None values
    trajectory_vector_len = 1
    for i in range(len(trajectory_vector_list)):
        if trajectory_vector_list[i] is not None:
            trajectory_vector_len = trajectory_vector_list[i].shape[0]
            break
    none_inds = []
    for i in range(len(trajectory_vector_list)):
        if trajectory_vector_list[i] is None:
            trajectory_vector_list[i] = np.zeros(trajectory_vector_len)
            none_inds.append(i)

    trajectory_vector_np = np.array(trajectory_vector_list)

    fusion_error_proxy_inds = get_fusion_error_proxy_inds(objectives_list_no_traj)
    print('len(fusion_error_proxy_inds)', len(fusion_error_proxy_inds))

    if len(fusion_error_proxy_inds) > 0:
        trajectory_vector_fusion_error_proxy_np = trajectory_vector_np[fusion_error_proxy_inds]

        objectives_list = []
        if traj_dist_metric in ['nearest', 'conditional_nearest']:
            kd_tree = KDTree(trajectory_vector_fusion_error_proxy_np, leaf_size=1)

        for i in range(trajectory_vector_np.shape[0]):
            if traj_dist_metric in ['nearest', 'conditional_nearest']:
                if len(kd_tree.data) > 1:
                    dists, inds = kd_tree.query(np.expand_dims(trajectory_vector_np[i], axis=0), k=2)
                    trajectory_vector_d = dists[0][1] / (route_interval_num*2)
                else:
                    trajectory_vector_d = 0
            elif traj_dist_metric == ['average', 'conditional_average']:
                diff = np.sum(np.abs(trajectory_vector_fusion_error_proxy_np - trajectory_vector_np[i]), axis=1)
                diff_avg = np.mean(diff)
                trajectory_vector_d = diff_avg / (route_interval_num*2)
            # print('trajectory_vector_d', trajectory_vector_d)
            # print('objectives_list_no_traj', objectives_list_no_traj)
            if traj_dist_metric in ['conditional_nearest', 'conditional_average'] and objectives_list_no_traj[i][5] < 0.05:
                trajectory_vector_d = 0
            if i in none_inds:
                trajectory_vector_d = 0
            objectives_list_no_traj[i][6] = trajectory_vector_d
            objectives_list.append(objectives_list_no_traj[i])
    else:
        objectives_list = objectives_list_no_traj



    job_results = objectives_list[-len(tmp_run_info_list):]
    # print('job_results', job_results)
    # print('x_sublist, job_results, trajectory_vector_sublist', x_sublist, job_results, trajectory_vector_sublist)


    return job_results, x_list, objectives_list, trajectory_vector_list
