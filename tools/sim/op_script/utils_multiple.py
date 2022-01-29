#!/usr/bin/env python3
import sys
import os

def append_relevant_paths():
    sys.path.append(os.path.expanduser('~/openpilot/tools/sim'))
    sys.path.append(os.path.expanduser('~/Documents/self-driving-cars/2020_CARLA_challenge'))

    carla_root = os.path.expanduser('~/Documents/self-driving-cars/carla_0911_no_rss')
    if not os.path.exists(carla_root):
        carla_root = os.path.expanduser('~/Documents/self-driving-cars/carla_0911_rss')

    sys.path.append(carla_root+'/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg')
    sys.path.insert(0, carla_root+'/PythonAPI/carla')
    sys.path.insert(0, carla_root+'/PythonAPI')










append_relevant_paths()
import carla
import math
import time
import pickle
import numpy as np
from matplotlib import pyplot as plt
RADAR_TO_CAMERA = 1.52
from tools.sim.op_script.object_types import vehicle_types


def generate_actor(world, client, blueprint_library, middle_points, vehicles_list, _vehicle_lights):
    blueprint_library = world.get_blueprint_library()
    batch = []
    actors_list = []
    fail = False
    for i, actor_i in enumerate(vehicles_list):
        middle_point = middle_points[i]

        forward = middle_point.rotation.get_forward_vector()
        right = middle_point.rotation.get_right_vector()

        location_offset = actor_i.x*forward + actor_i.y*right

        new_location = location_offset + middle_point.location
        new_rotation = carla.Rotation(pitch=middle_point.rotation.pitch, yaw=float(actor_i.yaw)+middle_point.rotation.yaw, roll=middle_point.rotation.roll)
        new_transform = carla.Transform(new_location, new_rotation)

        bp = blueprint_library.find(vehicle_types[actor_i.model])
        try:
            print(bp, new_transform)
            actor = world.spawn_actor(bp, new_transform)
        except:
            print('bp', bp, 'new_transform', new_transform, 'encounters collision!!!')
            fail = True
        actor.set_light_state(_vehicle_lights)
        actors_list.append(actor)

    return actors_list, fail


def activate_actors(spec_actors_list, actors_list, tm):
    for i in range(len(actors_list)):
        actors_list[i].set_autopilot(True, tm.get_port())
        perc_speed_diff = float(spec_actors_list[i].speed)
        obj_str = vehicle_types[spec_actors_list[i].model]

        tm.vehicle_percentage_speed_difference(actors_list[i], perc_speed_diff)
        tm.auto_lane_change(actors_list[i], False)

def activate_actors_batch(spec_actors_list, actors_list, tm, client):
    batch = []
    for i in range(len(actors_list)):
        # actors_list[i].set_autopilot(True, tm.get_port())
        batch.append(carla.command.SetAutopilot(actors_list[i].id, enabled=True))
    client.apply_batch_sync(batch, False)
    for i in range(len(actors_list)):
        perc_speed_diff = float(spec_actors_list[i].speed)
        obj_str = vehicle_types[spec_actors_list[i].model]

        tm.vehicle_percentage_speed_difference(actors_list[i], perc_speed_diff)
        tm.auto_lane_change(actors_list[i], False)
        # hack: try this for now
        # tm.ignore_vehicles_percentage(actors_list[i], 100)


def generate_actor_batch(world, client, blueprint_library, middle_points, vehicles_list, _vehicle_lights):

    blueprint_library = world.get_blueprint_library()
    batch = []
    actors_list = []
    for i, actor_i in enumerate(vehicles_list):
        middle_point = middle_points[i]

        forward = middle_point.rotation.get_forward_vector()
        right = middle_point.rotation.get_right_vector()

        location_offset = actor_i.x*forward + actor_i.y*right

        new_location = location_offset + middle_point.location
        new_rotation = carla.Rotation(pitch=middle_point.rotation.pitch, yaw=float(actor_i.yaw)+middle_point.rotation.yaw, roll=middle_point.rotation.roll)
        new_transform = carla.Transform(new_location, new_rotation)

        bp = blueprint_library.find(vehicle_types[actor_i.model])

        batch.append(carla.command.SpawnActor(bp, new_transform).then(carla.command.SetVehicleLightState(carla.command.FutureActor, _vehicle_lights)))

    fail = False
    for response in client.apply_batch_sync(batch, False):
        if response.error:
            print(response.error, 'bp', bp, 'new_transform', new_transform, 'encounters collision!!!')
            fail = True
        else:
            actors_list.append(world.get_actor(response.actor_id))

    return actors_list, fail


def change_lane(tm, vehicles_list, actors_list, prev_interval):
    for i, vehicle_i in enumerate(vehicles_list):
        if prev_interval < len(vehicle_i.behaviors):
            lane_change = vehicle_i.behaviors[prev_interval].lane_change
            perc_speed_diff = float(vehicle_i.behaviors[prev_interval].speed)
            if lane_change > 0:
                if lane_change == 1:
                    direction = False
                elif lane_change == 2:
                    direction = True
                else:
                    raise Exception('lane_change: '+str(lane_change))
                tm.force_lane_change(actors_list[i], direction)
            tm.vehicle_percentage_speed_difference(actors_list[i], perc_speed_diff)

# def activate_actors_batch(spec_actors_list, actors_list, tm):
#     batch = []
#     for actor in actors_list:
#         batch.append(SetAutopilot(actor.id, True, tm.get_port()))
#
#     for actor in actors_list:
#      tm.vehicle_percentage_speed_difference(actors_list[i], perc_speed_diff)
#      tm.auto_lane_change(actors_list[i], False)

def get_speed(actor):
    other_v = actor.get_velocity()
    speed = np.linalg.norm(np.array([other_v.x, other_v.y]))
    return speed



def dist(ego_car, other_actor):
    ego_loc = ego_car.get_location()
    other_loc = other_actor.get_location()
    d_abs = np.sqrt((ego_loc.x-other_loc.x)**2+(ego_loc.y-other_loc.y)**2)
    return d_abs


# modification
def destroy(client, sensors, actors_list):
    print("clean exit")
    if client is None:
        return
    tm = client.get_trafficmanager()
    world = client.get_world()
    import traceback
    try:
        print('destroy sensors')
        for sensor in sensors:
            sensor.stop()
            sensor.destroy()
        print('destroy actors')
        # client.apply_batch_sync([carla.command.SetAutopilot(x.id, False) for x in  world.get_actors()], False)
        # world.tick()
        client.apply_batch_sync([carla.command.DestroyActor(x) for x in  world.get_actors()], False)
        world.tick()

        if any((sensor.is_alive for sensor in sensors)) or any((actor.is_alive for actor in actors_list)):
            client.reload_world()
        print("done")
    except:
        traceback.print_exc()

    print('reset to async')
    settings = world.get_settings()
    tm.set_synchronous_mode(False)
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)


def print_and_write(f_out, str):
    f_out.write(str)
    print(str,sep='',end='')


def feed_in_commands(q, commands_list, remaining):
    if remaining > 0:
        for _ in range(remaining):
            commands_list.append(1)
    else:
        for _ in range(-remaining):
            commands_list.append(-1)

    for command in commands_list:
        if command == 1:
            print('put cruise up')
            q.put("cruise_up")
        elif command == -1:
            print('put cruise down')
            q.put("cruise_down")
        else:
            raise Exception('unknown command: '+str(command))
        time.sleep(0.1)

def keyboard_poll_thread_customized(q, q2, ego_maximum_speed, starting_sleep_time, starting_scenario_time):
    t_0 = time.time()
    q.put('time_'+str(t_0))
    time.sleep(starting_sleep_time+starting_scenario_time)

    remaining = 0
    print('remaining1', remaining)
    feed_in_commands(q, [1], remaining)

    while True:
        if not q2.empty():
            message = q2.get()
            m = message.split('_')
            if m[0] == 'end':
                print('message:', m)
                return
            elif m[0] == 'speed':
                v_diff = float(m[1])
                remaining = int(v_diff // 5)
                print('remaining2', remaining)
                feed_in_commands(q, [], remaining)

def keyboard_poll_thread_customized_sync(q, q2, ego_maximum_speed, starting_sleep_time, starting_scenario_time, fixed_delta_seconds):
    remaining = 0
    initialized = False
    while True:
        if not q2.empty():
            message = q2.get()
            m = message.split('_')
            if m[0] == 'counter':
                counter = int(m[1])
                if counter * fixed_delta_seconds > (starting_sleep_time+starting_scenario_time) and not initialized:
                    q.put('start-counter_'+str(counter))
                    feed_in_commands(q, [1, -1, 1, -1, 1, -1, 1], remaining)
                    initialized = True
            elif m[0] == 'end':
                print('message:', m)
                return
            elif m[0] == 'speed':
                v_diff = float(m[1])
                remaining = int(v_diff // 8)
                print('remaining2', remaining)
                feed_in_commands(q, [], remaining)


def brake_dist(speed):
    dBrake = 0.0467 * speed**2.0 + 0.4116 * speed - 1.9913 + 0.5
    if dBrake < 0:
        dBrake = 0
    return dBrake


def rotate_via_numpy(xy, radians):
    """Use numpy to build a rotation matrix and take the dot product."""
    x, y = xy
    c, s = np.cos(-radians), np.sin(-radians)
    j = np.array([[c, -s], [s, c]])
    m = np.dot(j, np.array([x, y]))

    return m[0], m[1]

def get_delta_d(ego, actor_i, maxint):
    ego_vel = ego.get_velocity()
    ego_speed = np.sqrt(ego_vel.x**2+ego_vel.y**2)
    ego_loc = ego.get_location()
    ego_yaw = ego.get_transform().rotation.yaw

    actor_i_loc = actor_i.get_location()


    ego_loc_x, ego_loc_y = rotate_via_numpy((ego_loc.x, ego_loc.y), np.deg2rad(ego_yaw))
    actor_i_loc_x, actor_i_loc_y = rotate_via_numpy((actor_i_loc.x, actor_i_loc.y), np.deg2rad(ego_yaw))

    d = np.sqrt((ego_loc_x - actor_i_loc_x)**2+(ego_loc_y-actor_i_loc_y)**2)
    d_minus_lengths = d - 4.6

    deltaD = maxint
    deltaDFront = maxint

    # When npc is in front
    if actor_i_loc_x  > ego_loc_x + 4.6:
        if ego_loc_y - 2 < actor_i_loc_y < ego_loc_y + 2:
            deltaDFront = d_minus_lengths - brake_dist(ego_speed)

    deltaD = deltaDFront

    return deltaD, d


# a more fine-grained version of get_delta_d
def get_delta_d_2(ego_car, actor_i, maxint, map):

    # check how many vertices inside the current lane
    ego_wp = map.get_waypoint(ego_car.get_location())
    ego_lane_id = ego_wp.lane_id

    other_actor_bbox = get_bbox(actor_i)
    vertices_in_lane = np.zeros([4])
    for i, vertex in enumerate(other_actor_bbox):
        other_wp = map.get_waypoint(vertex)
        other_lane_id = other_wp.lane_id
        if ego_lane_id == other_lane_id:
            vertices_in_lane[i] = 1

    ego_bx = ego_car.bounding_box.extent.x
    other_bx = actor_i.bounding_box.extent.x
    other_by = actor_i.bounding_box.extent.y


    ego_vel = ego_car.get_velocity()
    ego_speed = np.sqrt(ego_vel.x**2+ego_vel.y**2)
    ego_loc = ego_car.get_location()
    ego_yaw = ego_car.get_transform().rotation.yaw

    actor_i_loc = actor_i.get_location()


    ego_loc_x, ego_loc_y = rotate_via_numpy((ego_loc.x, ego_loc.y), np.deg2rad(ego_yaw))
    actor_i_loc_x, actor_i_loc_y = rotate_via_numpy((actor_i_loc.x, actor_i_loc.y), np.deg2rad(ego_yaw))


    dx = actor_i_loc_x - ego_loc_x
    dy = actor_i_loc_y - ego_loc_y
    d_center = np.sqrt(dx**2+dy**2)

    # design choice: subtract length only when >=1 rear vertex is in lane
    dx -= ego_bx
    if np.sum(vertices_in_lane[2:]) > 0:
        dx -= other_bx
    # design choice: adjust dy when <=2 vertices are in the lane
    if np.sum(vertices_in_lane) <= 2:
        if dy > 0:
            dy -= other_by
        else:
            dy += other_by

    d_minus_lengths = np.sqrt(dx**2+dy**2)
    deltaDFront = maxint

    # When npc is in front
    if dx > 0 and dy < 2:
        deltaDFront = d_minus_lengths - brake_dist(ego_speed)

    deltaD = deltaDFront

    return deltaD, d_center



def get_predicted_lead_msg(radard_data):
    lead_predcited_list = []
    for lead_pred in [radard_data.leadPredicted1, radard_data.leadPredicted2]:
        lead_pred_i = {
            'dRel': lead_pred.dRel,
            'yRel': lead_pred.yRel,
            'vRel': lead_pred.vRel,
            'modelProb': lead_pred.modelProb
        }
        lead_predcited_list.append(lead_pred_i)
    return lead_predcited_list


def get_camera_lead_msg(model_output):
    camera_leads = []
    l = np.min([2, len(model_output.leads)])
    for i in range(l):
        lead_msg = model_output.leads[i]
        lead_d_camera = {
            "dRel": float(lead_msg.xyva[0] - RADAR_TO_CAMERA),
            "yRel": float(-lead_msg.xyva[1]),
            "vRel": float(lead_msg.xyva[2]),
            "a": float(lead_msg.xyva[3]),

            "dRelStd": float(lead_msg.xyvaStd[0]),
            "yRelStd": float(lead_msg.xyvaStd[1]),
            "vRelStd": float(lead_msg.xyvaStd[2]),
            "aStd": float(lead_msg.xyvaStd[3]),

            "modelProb": float(lead_msg.prob),
            "t": float(lead_msg.t),
        }
        camera_leads.append(lead_d_camera)
    return camera_leads

def reformat_radar_clusters(radar_clusters):
    radar_clusters_reformated = []
    for radar_cluster in radar_clusters:
        radar_cluster_reformated = {
            # "trackId": int(radar_cluster.trackId),
            "dRel": float(radar_cluster.dRel),
            "yRel": float(radar_cluster.yRel),
            "vRel": float(radar_cluster.vRel),
            "modelProb": 1,
        }
        radar_clusters_reformated.append(radar_cluster_reformated)

    return radar_clusters_reformated

lead_d_carla_default = {
    "dRel": 0,
    "yRel": 0,
    "vRel": 0,
    "aLeadK": 0,
    "status": False,
    "modelProb": 0,
}




def get_lead_info_in_carla(ego_car, actors_list, map, verbose=False):
    ego_loc = ego_car.get_location()
    yaw = ego_car.get_transform().rotation.yaw
    ego_loc_x, ego_loc_y = rotate_via_numpy((ego_loc.x, ego_loc.y), np.deg2rad(yaw))

    dx_min = 130
    lead_d_carla = lead_d_carla_default
    ego_wp = map.get_waypoint(ego_car.get_location())
    ego_lane_id = ego_wp.lane_id

    chosen_other_actor = None
    for other_actor in actors_list:
        other_actor_bbox = get_bbox(other_actor)
        vertices_in_lane = np.zeros([4])
        for i, vertex in enumerate(other_actor_bbox):
            other_wp = map.get_waypoint(vertex)
            other_lane_id = other_wp.lane_id
            if ego_lane_id == other_lane_id:
                vertices_in_lane[i] = 1
            if verbose:
                # if other_actor.type_id == 'vehicle.bmw.grandtourer':
                #     print(other_actor.type_id, other_lane_id, ego_lane_id, vertex, other_wp)
                pass


        other_actor_loc = other_actor.get_location()
        other_actor_loc_x, other_actor_loc_y = rotate_via_numpy((other_actor_loc.x, other_actor_loc.y), np.deg2rad(yaw))

        dx = other_actor_loc_x - ego_loc_x
        dy = other_actor_loc_y - ego_loc_y

        # if np.abs(dy) < 2 and dx > 2.3 and dx < dx_min:
        if np.sum(vertices_in_lane) > 0 and dx > 0.5 and dx < dx_min:
            chosen_other_actor = other_actor
            dx_min = dx

            ego_v = ego_car.get_velocity()
            ego_v_x, ego_v_y = rotate_via_numpy((ego_v.x, ego_v.y), np.deg2rad(yaw))

            other_v = other_actor.get_velocity()
            other_v_x, other_v_y = rotate_via_numpy((other_v.x, other_v.y), np.deg2rad(yaw))
            other_a = other_actor.get_acceleration()
            other_a_x, other_a_y = rotate_via_numpy((other_a.x, other_a.y), np.deg2rad(yaw))
            other_a_total = np.sqrt(other_a_x**2+other_a_y**2)

            # sign_x = np.sign(dx)
            # sign_y = np.sign(dy)
            # v_dx_2 = (other_v_x - ego_v_x)**2 * sign_x
            # v_dy_2 = (other_v_y - ego_v_y)**2 * sign_y
            # dv_2_signed = v_dx_2 + v_dy_2
            # dv_2_sign = np.sign(dv_2_signed)
            # dv = np.sqrt(np.abs(dv_2_signed)) * dv_2_sign

            dv = (other_v_x - ego_v_x) * np.sign(dx)

            a = other_a_total

            ego_bx = ego_car.bounding_box.extent.x
            other_bx = other_actor.bounding_box.extent.x
            other_by = other_actor.bounding_box.extent.y

            # design choice: subtract length only when >=1 rear vertex is in lane
            dx -= ego_bx
            if np.sum(vertices_in_lane[2:]) > 0:
                dx -= other_bx
            # design choice: adjust dy when <=2 vertices are in the lane
            if np.sum(vertices_in_lane) <= 2:
                if dy > 0:
                    dy -= other_by
                else:
                    dy += other_by


            lead_d_carla = {
                "dRel": max([0., float(dx)]),
                "yRel": float(-dy),
                "vRel": float(dv),
                "aLeadK": float(a),
                "status": True,
                "modelProb": float(1.),
            }

    if verbose:
        if chosen_other_actor is not None:
            print(chosen_other_actor.type_id, 'is the leading vehicle!')
        else:
            print('No NPC vehicle is the leading vehicle!')
        print('lead_d_carla', lead_d_carla)

    return lead_d_carla

def get_bbox(vehicle):
    current_tra = vehicle.get_transform()
    current_loc = current_tra.location

    heading_vec = current_tra.get_forward_vector()
    heading_vec.z = 0
    heading_vec = heading_vec / math.sqrt(
        math.pow(heading_vec.x, 2) + math.pow(heading_vec.y, 2)
    )
    perpendicular_vec = carla.Vector3D(-heading_vec.y, heading_vec.x, 0)

    extent = vehicle.bounding_box.extent
    x_boundary_vector = heading_vec * extent.x
    y_boundary_vector = perpendicular_vec * extent.y

    bbox = [
        current_loc + carla.Location(x_boundary_vector - y_boundary_vector),
        current_loc + carla.Location(x_boundary_vector + y_boundary_vector),
        current_loc + carla.Location(-1 * x_boundary_vector - y_boundary_vector),
        current_loc + carla.Location(-1 * x_boundary_vector + y_boundary_vector),
    ]

    return bbox

def save_simulation_data(simulation_data, simulation_data_path, q2, end_condition, frame_id):
    simulation_data['end_condition'] = end_condition
    simulation_data['end_frame_id'] = frame_id
    # design choice: hack: this the saving happens while the counter keeps running. this is not thread-safe but given only more key, value pairs are added to the dictionary while it is pickled, this works for current purpose. need to fix in the future
    from copy import copy
    with open(simulation_data_path, 'wb') as f_out:
        pickle.dump(copy(simulation_data), f_out)
    q2.put('end_'+end_condition)

def save_simulation_data_sync2(simulation_data, simulation_data_path, end_condition, frame_id):
    simulation_data['end_condition'] = end_condition
    simulation_data['end_frame_id'] = frame_id

    from copy import copy
    with open(simulation_data_path, 'wb') as f_out:
        pickle.dump(copy(simulation_data), f_out)
    print('end_condition', end_condition)

def dist_between_locations(w1, w2):
    return np.sqrt((w1.x-w2.x)**2+(w1.y-w2.y)**2)

def get_vehicle_info(counter, frame_id, vehicle_control, vehicle_state, npc_actor_0_speed, cur_ego_car_model):

    vehicle_info = {
        'counter': counter,
        'frame_id': frame_id,
        'vehicle_control_throttle': vehicle_control.throttle, 'vehicle_control_steer': vehicle_control.steer,
        'vehicle_control_brake': vehicle_control.brake,
        'vehicle_state_speed': vehicle_state.speed,
        'vehicle_state_vel_x': vehicle_state.vel.x,
        'vehicle_state_vel_y': vehicle_state.vel.y,
        'vehicle_state_vel_z': vehicle_state.vel.z,
        'vehicle_state_angle': vehicle_state.angle,
        'vehicle_state_cruise_button': vehicle_state.cruise_button,
        'vehicle_state_is_engaged': vehicle_state.is_engaged,
        'npc_actor_0_speed': npc_actor_0_speed,
        'cur_ego_car_model': cur_ego_car_model
    }

    return vehicle_info
