#!/usr/bin/env python3

# addition:

from tools.sim.op_script.utils_multiple import append_relevant_paths
if __name__ == "__main__":
    append_relevant_paths()










import sys
import socket
import os
import argparse
import carla
import math
import numpy as np
import time
import pickle
import traceback
import copy
from cereal import log
from typing import Any

import cereal.messaging as messaging
from common.params import Params
from common.realtime import Ratekeeper, DT_DMON, frequency

from selfdrive.car.honda.values import CruiseButtons
from selfdrive.test.helpers import set_params_enabled

# addition:
from op_script.object_types import WEATHERS, car_types, large_car_types, motorcycle_types, cyclist_types, vehicle_colors
import shutil
from matplotlib import pyplot as plt
from distutils.dir_util import copy_tree
import multiprocessing
import threading
import queue
import zmq

from tools.sim.op_script.utils_multiple import generate_actor, activate_actors, get_lead_info_in_carla, destroy, print_and_write, dist, get_speed, get_delta_d, get_delta_d_2, lead_d_carla_default, get_camera_lead_msg, get_predicted_lead_msg, save_simulation_data, reformat_radar_clusters, get_vehicle_info, dist_between_locations, save_simulation_data_sync2, generate_actor_batch, change_lane, activate_actors_batch
from tools.sim.op_script.setup_labels_and_bounds import emptyobject
from tools.sim.op_script.op_specific import convert_x_to_customized_data, initialize_op_specific, check_bug, classify_bug_type, estimate_objectives, start_server, start_client, try_load_world, is_port_in_use, angle_from_center_view_fov, create_transform, estimate_trajectory_vector
from tools.sim.op_script.sensor_interface import panda_state_function, fake_driver_monitoring
from tools.sim.lib.can import can_function

from skimage.transform import resize
from sklearn.neighbors import KDTree

from customized_utils import make_hierarchical_dir, exit_handler


W, H = 1164, 874
REPEAT_COUNTER = 5
PRINT_DECIMATION = 100
STEER_RATIO = 15.
# addition: from openpilot/selfdrive/config.py
RADAR_TO_CAMERA = 1.52   # RADAR is ~ 1.5m ahead from center of mesh frame
# modification
pm = messaging.PubMaster(['roadCameraState', 'sensorEvents', 'can', "gpsLocationExternal", "leadCarla", "parametersFromCarla"])
# modification
# sm = messaging.SubMaster(['carControl','controlsState', 'modelV2'])
sm = messaging.SubMaster(['carControl','controlsState', 'modelV2', 'radardData', 'radarClusters', 'liveTracks'])





sensors_freq = {
            'camera_front': 0.05,
            'imu': 0,
            'gps': 0,
            'camera_top': 1,
            'radar': 0.05,
            'collision_detector': 0}


SAVE_DATA_PER_FRAMES = 1
SAVE_CAMERA_PER_FRAMES = 1

class VehicleState:
  def __init__(self):
    self.speed = 0
    self.angle = 0
    self.bearing_deg = 0.0
    self.vel = carla.Vector3D()
    self.cruise_button= 0
    self.is_engaged=False

    # addition:
    self.radar_data = []

def steer_rate_limit(old, new):
  # Rate limiting to 0.5 degrees per step
  limit = 0.5
  if new > old + limit:
    return old + limit
  elif new < old - limit:
    return old - limit
  else:
    return new

frame_id = 0
counter = 0
time_counter = -1


# save camera images in another thread to avoid sequential overhead
def save_camera_images(cam_save_q):
    while True:
        data = cam_save_q.get()
        if data is None:
            return

        sub_folder, frame_id, img = data
        # scale to 0.25 of the original scale
        image_resized = resize(img[:, :, [2, 1, 0]], (219, 291))
        plt.imsave(os.path.join(sub_folder, str(frame_id)+'.jpg'), image_resized)


def cam_callback(image, sub_folder, ego_car, actors_list, map, cam_save_q):

    global frame_id
    img = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    img = np.reshape(img, (H, W, 4))
    img = img[:, :, [0, 1, 2]].copy()
    # frame_id = image.frame
    frame_id = counter
    dat = messaging.new_message('roadCameraState')
    dat.roadCameraState = {
    "frameId": counter,
    "image": img.tobytes(),
    "transform": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    }
    pm.send('roadCameraState', dat)
    # print(counter, 'cam callback')

    lead_d_carla = get_lead_info_in_carla(ego_car, actors_list, map)
    dat = messaging.new_message("leadCarla")
    dat.leadCarla = lead_d_carla
    pm.send('leadCarla', dat)

    if frame_id % SAVE_CAMERA_PER_FRAMES == 0:
        cam_save_q.put((sub_folder, frame_id, img))
        time.sleep(0.005)

def other_cam_callback(image, sub_folder, cam_save_q):
    global frame_id
    if not os.path.exists(sub_folder):
        os.mkdir(sub_folder)
    img = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    img = np.reshape(img, (219, 291, 4))
    img = img[:, :, [0, 1, 2]].copy()

    if frame_id % SAVE_CAMERA_PER_FRAMES == 0:
        cam_save_q.put((sub_folder, frame_id, img))
        time.sleep(0.005)

def radar_callback(radar_data, world, vehicle, vehicle_state, dbscan_eps, dbscan_min_samples, visualize_radar=False):

    from sklearn.preprocessing import normalize
    from sklearn.cluster import KMeans, DBSCAN

    # ['closest', 'k-means clustering', 'dbscan clustering']
    mode = 'dbscan clustering'

    # To get a numpy [[vel, altitude, azimuth, depth],...[,,,]]:
    points_original = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
    points_original = np.reshape(points_original, (len(radar_data), 4))

    radar_data_cart_tmp = np.zeros((points_original.shape[0], 4))
    radar_data_cart = np.zeros((16, 4))
    radar_data_cart[:, 1] = 255

    if mode == 'closest':
        points_sorted = np.array(sorted(points_original.tolist(), key=lambda t:t[3]))
        points = points_sorted[:16]
    else:
        points = points_original
    current_rot = radar_data.transform.rotation

    for j, point in enumerate(points):
        velocity, azimuth, altitude, depth = point
        azi = math.degrees(azimuth)
        alt = math.degrees(altitude)

        fw_vec = carla.Vector3D(x=float(depth))

        carla.Transform(
            carla.Location(),
            carla.Rotation(
                pitch=alt,
                yaw=azi,
                roll=0)).transform(fw_vec)

        radar_data_cart_tmp[j, 0] = velocity
        radar_data_cart_tmp[j, 1] = fw_vec.x
        radar_data_cart_tmp[j, 2] = fw_vec.y
        radar_data_cart_tmp[j, 3] = fw_vec.z

    # [vel, long_dist, lat_dist]
    if 'clustering' in mode and len(radar_data_cart_tmp) > 0:
        # normalize each feature
        # radar_data_cart_tmp_norm = normalize(radar_data_cart_tmp, axis=0)

        radar_data_cart_tmp_norm = radar_data_cart_tmp[:, :3]

        if mode == 'k-means clustering':
            clustering = KMeans(n_clusters=16, random_state=0).fit(radar_data_cart_tmp_norm[:, :])
        elif mode == 'dbscan clustering':
            clustering = DBSCAN(eps=dbscan_eps, min_samples=dbscan_min_samples).fit(radar_data_cart_tmp_norm[:, :])
        else:
            raise Exception('unknown cluster method: '+mode)

        unique_labels = np.unique(clustering.labels_)
        cluster_num = len(unique_labels)
        # does not count the outlier class
        if -1 in unique_labels:
            cluster_num -= 1

        radar_data_cart_tmp_clusters = [[] for _ in range(cluster_num)]

        for i, label in enumerate(clustering.labels_):
            if label >= 0:
                radar_data_cart_tmp_clusters[label].append(radar_data_cart_tmp[i])

        chosen_cluster_num = np.min([cluster_num, 16])
        # TBD: currently keeping the largest top N clusters
        chosen_clusters = sorted(radar_data_cart_tmp_clusters, key=lambda cluster:-len(cluster))[:chosen_cluster_num]

        for j, cluster in enumerate(chosen_clusters):
            radar_data_cart[j, :] = np.mean(cluster, axis=0)

    else:
        radar_data_cart = radar_data_cart_tmp[:np.min([points_original.shape[0], 16])]

    vehicle_state.radar_data = radar_data_cart[:, :3]
    # print(counter, 'radar callback')
    if visualize_radar:
        for point in radar_data_cart:
            _, x, y, z = point
            fw_vec = carla.Vector3D(x=x, y=y, z=z)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch,
                    yaw=current_rot.yaw,
                    roll=current_rot.roll)).transform(fw_vec)

            p_t = radar_data.transform.location + fw_vec

            c = carla.Color(255, 0, 0)
            world.debug.draw_point(
                p_t,
                size=0.2,
                life_time=0.08,
                persistent_lines=False,
                color=c)

def collision_detector_callback(collision_info, tmp_data_file, simulation_data, simulation_data_path, accident_q):
    loc = collision_info.transform.location
    rot = collision_info.transform.rotation
    type_id = collision_info.other_actor.type_id
    vel = collision_info.actor.get_velocity()
    collision_speed = np.sqrt(vel.x**2+vel.y**2)
    d_angle_norm, _ = angle_from_center_view_fov(collision_info.other_actor.get_location(), collision_info.actor, fov=70)


    simulation_data['collision'] = {
        'x': loc.x,
        'y': loc.y,
        'z': loc.z,
        'pitch': rot.pitch,
        'yaw': rot.yaw,
        'roll': rot.roll,
        'type_id': type_id,
        'speed': collision_speed,
        'd_angle_norm': d_angle_norm,
        'frame_id': frame_id,
        'counter': counter
        }

    with open(tmp_data_file, 'a') as f_out:
        str_to_save = ','.join([str(v) for v in ('collision', loc.x, loc.y, loc.z, rot.pitch, rot.yaw, rot.roll, type_id, collision_speed, d_angle_norm)])+'\n'
        f_out.write(str_to_save)

    accident_q.put('collision')
    time.sleep(0.2)
    save_simulation_data_sync2(simulation_data, simulation_data_path, 'collision', frame_id)




def imu_callback(imu, vehicle_state):

  vehicle_state.bearing_deg = math.degrees(imu.compass)
  dat = messaging.new_message('sensorEvents', 2)
  dat.sensorEvents[0].sensor = 4
  dat.sensorEvents[0].type = 0x10
  dat.sensorEvents[0].init('acceleration')
  dat.sensorEvents[0].acceleration.v = [imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z]
  # copied these numbers from locationd
  dat.sensorEvents[1].sensor = 5
  dat.sensorEvents[1].type = 0x10
  dat.sensorEvents[1].init('gyroUncalibrated')
  dat.sensorEvents[1].gyroUncalibrated.v = [imu.gyroscope.x, imu.gyroscope.y, imu.gyroscope.z]
  pm.send('sensorEvents', dat)
  # print(counter, 'imu callback')

def gps_callback(gps, vehicle_state):
  dat = messaging.new_message('gpsLocationExternal')

  # transform vel from carla to NED
  # north is -Y in CARLA
  velNED = [
    -vehicle_state.vel.y, # north/south component of NED is negative when moving south
    vehicle_state.vel.x, # positive when moving east, which is x in carla
    vehicle_state.vel.z,
  ]
  dat.gpsLocationExternal = {
    "flags": 1, # valid fix
    "verticalAccuracy": 1.0,
    "speedAccuracy": 0.1,
    "vNED": velNED,
    "bearingDeg": vehicle_state.bearing_deg,
    "latitude": gps.latitude,
    "longitude": gps.longitude,
    "altitude": gps.altitude,
    "source": log.GpsLocationData.SensorSource.ublox,
  }
  pm.send('gpsLocationExternal', dat)

def callback(data, tag, q):
    q.put({'tag': tag, 'data': data})


def retrieve_and_send(queues, fixed_delta_seconds, counter, world, ego_car, front_folder, overview_folder, tmp_data_file, simulation_data, simulation_data_path, actors_list, world_map, vehicle_state, world_frame, cam_save_q, dbscan_eps, dbscan_min_samples, visualize_radar):
    k = {}

    for tag, q in queues:
        while True:
            try:
                timeout = 0.005
                data = q.get(timeout=timeout)
                k[tag] = data
                break
            except:
                break

    for tag, data in k.items():
        # print(counter, tag, data.frame)
        if tag == 'camera_front':
            cam_callback(data, front_folder, ego_car, actors_list, world_map, cam_save_q)
        elif tag == 'imu':
            imu_callback(data, vehicle_state)
        elif tag == 'gps':
            gps_callback(data, vehicle_state)
        elif tag == 'camera_top':
            other_cam_callback(data, overview_folder, cam_save_q)
        elif tag == 'radar':
            radar_callback(data, world, ego_car, vehicle_state, dbscan_eps, dbscan_min_samples, visualize_radar=visualize_radar)

def set_up_sensors(blueprint_library, world, ego_car, tmp_data_file, simulation_data, simulation_data_path, accident_q, radar_hfov, radar_vfov, radar_range, radar_points_per_second):
    W, H = 1164, 874

    queues = []
    def make_queue(register_event, sensor_name):
        q = queue.Queue()
        register_event(q.put)
        queues.append((sensor_name, q))

    vehicle_state = VehicleState()

    # front camera:
    blueprint = blueprint_library.find('sensor.camera.rgb')
    blueprint.set_attribute('image_size_x', str(W))
    blueprint.set_attribute('image_size_y', str(H))
    blueprint.set_attribute('fov', '70')
    blueprint.set_attribute('sensor_tick', str(sensors_freq['camera_front']))
    transform = carla.Transform(carla.Location(x=0.8, z=1.13))
    camera_front = world.spawn_actor(blueprint, transform, attach_to=ego_car)
    # camera_front.listen(lambda data: callback(data, 'camera_front', q))


    # reenable IMU:
    imu_bp = blueprint_library.find('sensor.other.imu')
    imu = world.spawn_actor(imu_bp, transform, attach_to=ego_car)
    # imu.listen(lambda data: callback(data, 'imu', q))


    # GPS:
    gps_bp = blueprint_library.find('sensor.other.gnss')
    gps = world.spawn_actor(gps_bp, transform, attach_to=ego_car)
    # gps.listen(lambda data: callback(data, 'gps', q))


    # top camera
    blueprint = blueprint_library.find('sensor.camera.rgb')
    blueprint.set_attribute('image_size_x', str(291))
    blueprint.set_attribute('image_size_y', str(219))
    blueprint.set_attribute('fov', '120')
    blueprint.set_attribute('sensor_tick', str(sensors_freq['camera_top']))
    transform = carla.Transform(carla.Location(x=0, z=10), carla.Rotation(pitch=270))
    camera_top = world.spawn_actor(blueprint, transform, attach_to=ego_car)
    # camera_top.listen(lambda data: callback(data, 'camera_top', q))

    # radar
    blueprint = blueprint_library.find('sensor.other.radar')
    blueprint.set_attribute('horizontal_fov', str(radar_hfov)) #30, 90
    blueprint.set_attribute('vertical_fov', str(radar_vfov)) # 1, 3
    blueprint.set_attribute('range', str(radar_range)) #174, 60
    blueprint.set_attribute('sensor_tick', str(sensors_freq['radar']))
    blueprint.set_attribute('points_per_second', str(radar_points_per_second)) #12000, 24000
    transform = carla.Transform(carla.Location(x=2.32, z=1.13))
    radar = world.spawn_actor(blueprint, transform, attach_to=ego_car)
    # radar.listen(lambda data: callback(data, 'radar', q))

    # collision detector
    blueprint = blueprint_library.find('sensor.other.collision')
    transform = carla.Transform()
    collision_detector = world.spawn_actor(blueprint, transform, attach_to=ego_car)





    collision_detector.listen(lambda data: collision_detector_callback(data, tmp_data_file, simulation_data, simulation_data_path, accident_q))

    sensors = [camera_front, imu, gps, camera_top, radar, collision_detector]

    sensor_names = list(sensors_freq.keys())

    for sensor_name, sensor in zip(sensor_names[:-1], sensors[:-1]):
        make_queue(sensor.listen, sensor_name)
    return sensors, vehicle_state, queues



def bridge(customized_data, arguments, sim_specific_arguments, launch_server, port):
  t_0 = time.time()

  global counter
  global frame_id

  launch_openpilot_in_place = arguments.launch_openpilot_in_place




  threads = []
  end_q = queue.Queue()
  cam_save_q = queue.Queue()

  client = None
  sensors = []
  actors_list = []
  try:
      # connect to OP in terms of sec_since_boot
      context = zmq.Context.instance()
      time_socket = context.socket(zmq.PAIR)
      docker_ip = "localhost"
      time_socket.connect("tcp://" + docker_ip + ":5561")

      # launch OP
      if launch_openpilot_in_place:
          import subprocess
          import signal
          import pathlib
          folder = str(pathlib.Path(__file__).parent.resolve())
          print('folder', folder)
          pro = subprocess.Popen('bash '+folder+'/launch_openpilot2.sh', stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)

      # Initialization
      warm_up_time = arguments.warm_up_time
      episode_sim_time = arguments.episode_sim_time
      consider_trajectory_vector = arguments.consider_trajectory_vector
      dbscan_eps = arguments.dbscan_eps
      dbscan_min_samples = arguments.dbscan_min_samples
      visualize_radar = arguments.visualize_radar
      behavior_change_time_interval = (episode_sim_time - warm_up_time) // customized_data['num_of_vehicle_behavior_changes']
      prev_interval = -1
      cur_interval = 0

      simulation_data = {
        'collision': None,
        'camera_leads_dict': {},
        'radar_clusters_dict': {},
        'radar_clusters_raw_dict': {}, # for reference, small delay is ok
        'lead_d_carla_dict': {},
        'leads_predicted_list_dict': {},
        'd_avg_list_dict': {}, # small delay is ok
        'd_avg_old_list_dict': {}, # small delay is ok, for comparison purpose
        'vehicle_info_dict': {}, # for reference, small delay is ok
        'trajectory_dict': {}, # small delay is ok
        'end_condition': None,
        'end_frame_id': 0
      }

      town_name = sim_specific_arguments.route_info["town_name"]
      fixed_delta_seconds = arguments.fixed_delta_seconds
      start_transform_values = sim_specific_arguments.route_info["location_list"][0]
      end_transform_values = sim_specific_arguments.route_info["location_list"][1]

      ego_car_model = arguments.ego_car_model
      temporary_running_folder = arguments.temporary_running_folder
      simulation_data_path = arguments.simulation_data_path
      waypoint_interval_dist = arguments.waypoint_interval_dist
      trajectory_freq = arguments.trajectory_freq

      radar_hfov = arguments.radar_hfov
      radar_vfov = arguments.radar_vfov
      radar_range = arguments.radar_range
      radar_points_per_second = arguments.radar_points_per_second


      if os.path.exists(os.path.join(temporary_running_folder, 'model_output.txt')):
          os.remove(os.path.join(temporary_running_folder, 'model_output.txt'))
      front_folder = make_hierarchical_dir([temporary_running_folder, 'front'])
      overview_folder = make_hierarchical_dir([temporary_running_folder, 'top'])
      tmp_data_file = os.path.join(temporary_running_folder, 'tmp_data.txt')


      try:
          # Start CARLA server and client
          if launch_server or not is_port_in_use(int(port)):
              start_server(port, sim_specific_arguments.carla_path)

          if sim_specific_arguments.client is None:
              client = start_client("localhost", port)
              world, client = try_load_world(town_name, client, "localhost", port, sim_specific_arguments.carla_path)
              # sim_specific_arguments.client = client
          else:
              print('use previous client')
              client = sim_specific_arguments.client
              world = client.get_world()


              # world, client = try_load_world(town_name, client, "localhost", port, sim_specific_arguments.carla_path)

          # Set CARLA synchronous mode and Initialize synchronous traffic manager
          tm = client.get_trafficmanager()
          tm.set_random_device_seed(arguments.random_seed)
          settings = world.get_settings()
          print('set tm to sync')
          tm.set_synchronous_mode(True)
          # tm.set_hybrid_physics_mode(True)
          # tm.set_hybrid_physics_radius(50.0)
          settings.synchronous_mode = True
          settings.fixed_delta_seconds = fixed_delta_seconds
          print('apply settings')
          world_frame = world.apply_settings(settings)
          blueprint_library = world.get_blueprint_library()
          world_map = world.get_map()
          print('start')
          failure = False

      except:
          traceback.print_exc()
          raise


      # Initialize weather
      weather = carla.WeatherParameters(*customized_data['weather'])
      world.set_weather(weather)
      if weather.sun_altitude_angle < 0.0:
          _vehicle_lights =  carla.VehicleLightState(carla.VehicleLightState.LowBeam | carla.VehicleLightState.Position)
      else:
          _vehicle_lights = carla.VehicleLightState.NONE

      # town04
      # end_transform_values = (292.620300,-383.606567, 0.02, 0., -90, 0.)
      # town06
      # end_transform_values = (586.908142-160, -24.062836+3.5, 0.3, 0., -179.580566, 0.)
      # Initialize Ego Car
      ego_start_position = create_transform(start_transform_values)
      ego_end_position = create_transform(end_transform_values)



      # get waypoints on the path
      if launch_server or sim_specific_arguments.kd_tree is None and consider_trajectory_vector:

          start_waypoint = world_map.get_waypoint(ego_start_position.location)
          end_waypoint = world_map.get_waypoint(ego_end_position.location)
          cur_waypoint = start_waypoint
          route_loc_list = []
          while cur_waypoint.transform.location.distance(end_waypoint.transform.location) > 5:
              # print('cur_waypoint.transform', cur_waypoint.transform)
              cur_loc = cur_waypoint.transform.location
              route_loc_list.append((cur_loc.x, cur_loc.y))
              cur_waypoint = cur_waypoint.next(waypoint_interval_dist)[0]
          print('len(route_loc_list)', len(route_loc_list))

          sim_specific_arguments.kd_tree = KDTree(np.array(route_loc_list), leaf_size=1)





      ego_car_bp = blueprint_library.filter('vehicle.tesla.*')[1]
      ego_car_bp.set_attribute('role_name', 'hero')
      ego_car = world.spawn_actor(ego_car_bp, ego_start_position)
      ego_car.set_light_state(_vehicle_lights)

      max_steer_angle = ego_car.get_physics_control().wheels[0].max_steer_angle
      physics_control = ego_car.get_physics_control()
      physics_control.mass = 2326
      physics_control.torque_curve = [[20.0, 500.0], [5000.0, 500.0]]
      physics_control.gear_switch_time = 0.0
      ego_car.apply_physics_control(physics_control)


      # initialize NPC vehicles
      middle_points = [ego_start_position] * len(customized_data["vehicles_list"])
      for k, v in customized_data["customized_center_transforms"].items():
          if v[0] == "absolute_location":
              middle_point_i = create_transform(v[1:])
              ind = int(k.split('_')[-1])
              if ind < len(middle_points):
                  middle_points[ind] = middle_point_i
              else:
                  print('Warning:', ind, 'goes beyond the range of middle_points')
          else:
              print("unknown key", k)

      actors_list, fail = generate_actor_batch(world, client, blueprint_library, middle_points, customized_data["vehicles_list"], _vehicle_lights)
      if fail:
          save_simulation_data_sync2(simulation_data, simulation_data_path, 'actor_generation', frame_id)
          return

      # tick to spawn vehicles,
      for _ in range(100):
          world.tick()


      # Set up sensors
      accident_q = queue.Queue()
      sensors, vehicle_state, queues = set_up_sensors(blueprint_library, world, ego_car, tmp_data_file, simulation_data, simulation_data_path, accident_q, radar_hfov, radar_vfov, radar_range, radar_points_per_second)

      scenario_activated = False

      # tick to generate sensors and activate NPC vehicles
      world.tick()



      # launch fake car threads
      threads.append(threading.Thread(target=panda_state_function, args=(end_q,), daemon=True))
      threads.append(threading.Thread(target=fake_driver_monitoring, args=(end_q,), daemon=True))
      threads.append(threading.Thread(target=save_camera_images, args=(cam_save_q,), daemon=True))
      for thread in threads:
          thread.start()


      # init
      throttle_ease_out_counter = REPEAT_COUNTER
      brake_ease_out_counter = REPEAT_COUNTER
      steer_ease_out_counter = REPEAT_COUNTER

      vc = carla.VehicleControl(throttle=0, steer=0, brake=0, reverse=False)

      throttle_out = steer_out = brake_out = 0
      throttle_op = steer_op = brake_op = 0
      throttle_manual = steer_manual = brake_manual = 0

      old_steer = old_brake = old_throttle = 0
      throttle_manual_multiplier = 0.7 #keyboard signal is always 1
      brake_manual_multiplier = 0.7 #keyboard signal is always 1
      steer_manual_multiplier = 45 * STEER_RATIO  #keyboard signal is always 1

      # addition:
      saved_frame_id = -1
      min_d_avg = 130
      min_d_angle_norm = 1
      last_v_adjust_time = 0
      model_output = None


      is_openpilot_engaged = False
      op_start = False

      while 1:
        # -1. Check end conditions
        # 0. Send current time, tick the world, and send sensor data
        # 1. Read the throttle, steer and brake from op or manual controls
        # 2. Set instructions in Carla
        # 3. Send current carstate to op via can
        # if counter >= 18:
        #     break
        cur_time = counter * fixed_delta_seconds
        # --------------Step -1-------------------------------
        # Check end conditions
        # 1.timeout
        if cur_time >= episode_sim_time:
            save_simulation_data_sync2(simulation_data, simulation_data_path, 'timeout', frame_id)
            print('\n'*3, 'final transform:', ego_car.get_transform(), '\n'*3)
            return
        # 2.destination
        if dist_between_locations(ego_car.get_location(), ego_end_position.location) < 2:
            save_simulation_data_sync2(simulation_data, simulation_data_path, 'reach_destination', frame_id)
            return
        # 3.collision
        if not accident_q.empty():
            return
        # 4.out-of-line
        current_lane_id = world_map.get_waypoint(ego_car.get_location()).lane_id
        if counter == 0:
            prev_lane_id = current_lane_id
        else:
            if current_lane_id != prev_lane_id:
                save_simulation_data_sync2(simulation_data, simulation_data_path, 'out_of_lane', frame_id)
                return



        if counter % 5 == 0:
            if sim_specific_arguments.ego_car_model_final is not None and cur_time >= sim_specific_arguments.ego_final_start_time:
                cur_ego_car_model = sim_specific_arguments.ego_car_model_final
            else:
                cur_ego_car_model = ego_car_model
            dat = messaging.new_message("parametersFromCarla")
            dat.parametersFromCarla = {'fusion': cur_ego_car_model, 'targetSpeed': int(arguments.ego_maximum_speed)}
            pm.send('parametersFromCarla', dat)


        if counter % (1/(trajectory_freq*fixed_delta_seconds)) == 0:
            ego_loc = ego_car.get_location()
            ego_vel = ego_car.get_velocity()
            # m/s
            ego_speed = np.sqrt(ego_vel.x**2+ego_vel.y**2)

            simulation_data['trajectory_dict'][counter] = (ego_loc.x, ego_loc.y, ego_speed)
            # print(counter, ego_loc.x, ego_loc.y, ego_speed)
        # --------------Step 0-------------------------------
        # Send current time, tick the world, and send sensor data
        time_socket.send_string(str(counter))
        time_socket.recv()
        # if counter % 1 == 0:
        #     print(counter, 'time_step 0.1', time.time()-t_0)
        world_frame = world.tick()
        # if counter % 1 == 0:
        #     print(counter, 'time_step 0.2', time.time()-t_0)
        retrieve_and_send(queues, fixed_delta_seconds, counter, world, ego_car, front_folder, overview_folder, tmp_data_file, simulation_data, simulation_data_path, actors_list, world_map, vehicle_state, world_frame, cam_save_q, dbscan_eps, dbscan_min_samples, visualize_radar)
        # if counter % 1 == 0:
        #     print(counter, 'time_step 0.3', time.time()-t_0)
        # --------------Step 1-------------------------------
        # Initialize control
        throttle_out = steer_out = brake_out = 0.0
        throttle_op = steer_op = brake_op = 0
        throttle_manual = steer_manual = brake_manual = 0.0
        cruise_button = 0

        if not is_openpilot_engaged:
            cruise_button = CruiseButtons.RES_ACCEL
            is_openpilot_engaged = True

        if is_openpilot_engaged:
          carControl_updated = False
          controlsState_updated = False

          inner_count = 0
          while True:
              if inner_count >= 5:
                  raise
              can_function(pm, vehicle_state.speed, vehicle_state.angle, counter, vehicle_state.cruise_button, vehicle_state.is_engaged, vehicle_state.radar_data)
              sm.update(0)

              if sm.updated['carControl']:
                  carControl_updated = True
              if sm.updated['controlsState']:
                  controlsState_updated = True
              # print(counter, inner_count, carControl_updated, controlsState_updated, sm.updated['modelV2'])
              # print(sm['carControl'])
              # print(sm['controlsState'])
              if carControl_updated and controlsState_updated:
                  break
              time.sleep(0.5)
              inner_count += 1


          # print(sm['carControl'].actuators.gas, sm['carControl'].actuators.brake, sm['controlsState'].steeringAngleDesiredDeg)

          throttle_op = sm['carControl'].actuators.gas #[0,1]
          brake_op = sm['carControl'].actuators.brake #[0,1]
          steer_op = sm['controlsState'].steeringAngleDesiredDeg # degrees [-180,180]
          model_output = sm['modelV2']
          radard_data = sm['radardData']
          v_cruise = sm['controlsState'].vCruise
          radar_clusters = sm['radarClusters']
          radar_clusters_raw = sm['liveTracks']

          throttle_out = throttle_op
          steer_out = steer_op
          brake_out = brake_op
          steer_out = steer_rate_limit(old_steer, steer_out)
          old_steer = steer_out

          if v_cruise != arguments.ego_maximum_speed and counter * fixed_delta_seconds > (last_v_adjust_time + 0.2):
              last_v_adjust_time = counter * fixed_delta_seconds
              v_cruise = np.ceil(v_cruise // 8) * 8
              v_diff = arguments.ego_maximum_speed - v_cruise

              if v_diff // 8 > 0:
                  cruise_button = CruiseButtons.RES_ACCEL
              elif v_diff // 8 < 0:
                  cruise_button = CruiseButtons.DECEL_SET


        # --------------Step 2-------------------------------
        steer_carla = steer_out / (max_steer_angle * STEER_RATIO * -1)
        steer_carla = np.clip(steer_carla, -1,1)
        steer_out = steer_carla * (max_steer_angle * STEER_RATIO * -1)
        old_steer = steer_carla * (max_steer_angle * STEER_RATIO * -1)

        vc.throttle = throttle_out/0.6
        vc.steer = steer_carla
        vc.brake = brake_out
        if vc.throttle > 0.0 and not op_start:
            # print(counter, 'OP starts')
            pass

        if cur_time > warm_up_time:
            if not scenario_activated:
                activate_actors_batch(customized_data["vehicles_list"], actors_list, tm, client)
                scenario_activated = True
                print('vc.throttle', vc.throttle)
            op_start = True
        if op_start:
            ego_car.apply_control(vc)


        # --------------Step 3-------------------------------
        ego_vel = ego_car.get_velocity()
        # modification: remove ego_vel in the calculation of ego_speed
        ego_speed = math.sqrt(ego_vel.x**2 + ego_vel.y**2) # in m/s
        vehicle_state.speed = ego_speed
        vehicle_state.vel = ego_vel
        vehicle_state.angle = steer_out
        vehicle_state.cruise_button = cruise_button
        vehicle_state.is_engaged = is_openpilot_engaged


        # Save data
        model_output_frame_id = -1
        if model_output is not None:
            model_output_frame_id = int(model_output.frameId)


        if radard_data.frameId > 0 and radard_data.frameId % SAVE_DATA_PER_FRAMES == 0 and is_openpilot_engaged:
            # print('radard_data.frameId', radard_data.frameId)
            leads_predicted_list = get_predicted_lead_msg(radard_data)
            simulation_data['leads_predicted_list_dict'][radard_data.frameId] = leads_predicted_list

        # radard(modelV2) 20Hz
        if len(radar_clusters) > 0 and radar_clusters[0].frameId > 0 and radar_clusters[0].frameId % SAVE_DATA_PER_FRAMES == 0 and is_openpilot_engaged:
            # print('radar_clusters[0].frameId', radar_clusters[0].frameId)
            radar_clusters_reformated = reformat_radar_clusters(radar_clusters)
            simulation_data['radar_clusters_dict'][model_output_frame_id] = radar_clusters_reformated

            radar_clusters_raw_reformated = reformat_radar_clusters(radar_clusters_raw)
            simulation_data['radar_clusters_raw_dict'][model_output_frame_id] = radar_clusters_raw_reformated


        # camera(radardData) 20Hz
        if model_output_frame_id > 0 and model_output_frame_id % SAVE_DATA_PER_FRAMES == 0 and is_openpilot_engaged:
            camera_leads = get_camera_lead_msg(model_output)
            simulation_data['camera_leads_dict'][model_output_frame_id] = camera_leads

        # Carla GT (independent of OP), vehicle_info
        if counter % SAVE_DATA_PER_FRAMES == 0 and is_openpilot_engaged:
            if len(actors_list) > 0:
                lead_d_carla = get_lead_info_in_carla(ego_car, actors_list, world_map)
            else:
                lead_d_carla = [lead_d_carla_default, lead_d_carla_default]

            d_avg_old_list = []
            d_avg_list = []
            for actor_i in actors_list:
                deltaD, d = get_delta_d(ego_car, actor_i, maxint=130)
                d_avg_old = deltaD * 0.5 + d * 0.5
                d_avg_old_list.append(d_avg_old)

                deltaD, d = get_delta_d_2(ego_car, actor_i, maxint=130, map=world_map)
                d_avg = deltaD * 0.5 + d * 0.5
                d_avg_list.append(d_avg)

            simulation_data['lead_d_carla_dict'][frame_id] = lead_d_carla
            simulation_data['d_avg_old_list_dict'][frame_id] = d_avg_old_list
            simulation_data['d_avg_list_dict'][frame_id] = d_avg_list

            # record vehicle info
            npc_actor_0 = world.get_actors().filter('vehicle.*')[1]
            npc_actor_0_speed = get_speed(npc_actor_0)
            vehicle_info = get_vehicle_info(counter, frame_id, vc, vehicle_state, npc_actor_0_speed, cur_ego_car_model)

            simulation_data['vehicle_info_dict'][counter] = vehicle_info

        # Lane changes
        if scenario_activated:
            if behavior_change_time_interval == 0:
                cur_interval = 0
            else:
                cur_interval = int((cur_time - warm_up_time) // behavior_change_time_interval)
            if cur_interval > prev_interval:
                change_lane(tm, customized_data['vehicles_list'], actors_list, prev_interval)
                prev_interval = cur_interval

        counter += 1

  finally:
      print('exit gracefully')
      if launch_openpilot_in_place:
           # kill the OP prcess
           os.killpg(os.getpgid(pro.pid), signal.SIGTERM)

      end_q.put(None)
      cam_save_q.put(None)
      for thread in threads:
          print('stop thread')
          # thread.terminate()
          thread.join()
      counter = 0
      frame_id = 0
      destroy(client, sensors, actors_list)

      print('close zmq socket')
      time_socket.close(2)
      context.term()

      if launch_openpilot_in_place:
          # extra time to successfully stop OP processes
          time.sleep(5)
      print('\n', 'time this run:', time.time()-t_0, '\n')  # except






def run_op_simulation(x, fuzzing_content, fuzzing_arguments, sim_specific_arguments, dt_arguments, launch_server, sim_counter, port):

    launch_openpilot_in_place = True
    episode_sim_time = 24
    fixed_delta_seconds = frequency
    warm_up_time = 4
    # Note: this is independent of the one in rerun_carla_op.py
    pre_crash_window_length = 2.5
    consider_trajectory_vector = True
    speed_grid_num = 10
    waypoint_interval_dist = 5 # about ~30
    trajectory_freq = 5
    visualize_radar = False

    radar_hfov = 30 #30, 90
    radar_vfov = 1 # 1, 3
    radar_range = 174 #174, 60
    radar_points_per_second = 18000 # 18000, 36000

    dbscan_eps = 0.5
    dbscan_min_samples = 5


    # addition: folder managements
    parent_folder = fuzzing_arguments.parent_folder
    if not os.path.exists(parent_folder):
        os.mkdir(parent_folder)
    temporary_running_folder = os.path.join(parent_folder, "current_run_data")
    if os.path.exists(temporary_running_folder):
        shutil.rmtree(temporary_running_folder)
    os.mkdir(temporary_running_folder)

    simulation_data_path = os.path.join(temporary_running_folder, 'simulation_data.pickle')



    customized_data = convert_x_to_customized_data(x, fuzzing_content, port)
    delay_time_to_start = customized_data['delay_time_to_start']
    # hack: round ego_maximum_speed to multiple of 8
    ego_maximum_speed = (customized_data['ego_maximum_speed'] // 8) * 8
    num_of_vehicle_behavior_changes = customized_data['num_of_vehicle_behavior_changes']


    if hasattr(fuzzing_arguments, 'random_seed'):
        random_seed = fuzzing_arguments.random_seed
    else:
        random_seed = 0


    arguments = emptyobject(episode_sim_time=episode_sim_time, temporary_running_folder=temporary_running_folder, ego_maximum_speed=ego_maximum_speed, ego_car_model=fuzzing_arguments.ego_car_model, fixed_delta_seconds=fixed_delta_seconds,
    simulation_data_path=simulation_data_path, launch_openpilot_in_place=launch_openpilot_in_place, warm_up_time=warm_up_time,
    consider_trajectory_vector=consider_trajectory_vector,
    waypoint_interval_dist=waypoint_interval_dist,
    trajectory_freq=trajectory_freq, visualize_radar=visualize_radar, radar_hfov=radar_hfov, radar_vfov=radar_vfov, radar_range=radar_range, radar_points_per_second=radar_points_per_second, random_seed=random_seed, dbscan_eps=dbscan_eps, dbscan_min_samples=dbscan_min_samples)



    # make sure params are in a good state
    set_params_enabled()
    msg = messaging.new_message('liveCalibration')
    msg.liveCalibration.validBlocks = 20
    msg.liveCalibration.rpyCalib = [0.0, 0.0, 0.0]
    Params().put("CalibrationParams", msg.to_bytes())



    # print('before sim_specific_arguments.kd_tree', sim_specific_arguments.kd_tree)
    # print('before sim_specific_arguments.client', sim_specific_arguments.client)
    bridge(customized_data, arguments, sim_specific_arguments, launch_server, port)
    # print('after sim_specific_arguments.kd_tree', sim_specific_arguments.kd_tree)
    # print('after sim_specific_arguments.client', sim_specific_arguments.client)


    # Postprocess collected data
    last_n_frame_id = pre_crash_window_length / fixed_delta_seconds
    objectives = estimate_objectives(simulation_data_path, arguments.temporary_running_folder, last_n_frame_id)

    # get trajectory vector
    if sim_specific_arguments.kd_tree is not None:
        trajectory_vector = estimate_trajectory_vector(sim_specific_arguments.kd_tree, simulation_data_path, customized_data['ego_maximum_speed'], speed_grid_num)
    else:
        trajectory_vector = None


    # more folder management and save running data
    is_bug = check_bug(objectives)
    if is_bug:
        bug_type, bug_str = classify_bug_type(objectives, None)
    else:
        bug_type, bug_str = None, None
    if is_bug:
        if hasattr(fuzzing_arguments, 'mean_objectives_across_generations_path'):
            with open(fuzzing_arguments.mean_objectives_across_generations_path, 'a') as f_out:
                f_out.write(str(sim_counter)+','+bug_str+'\n')

    bug_folder = make_hierarchical_dir([parent_folder, 'bugs'])
    non_bug_folder = make_hierarchical_dir([parent_folder, 'non_bugs'])
    if is_bug:
        cur_folder = make_hierarchical_dir([bug_folder, str(sim_counter)])
    else:
        cur_folder = make_hierarchical_dir([non_bug_folder, str(sim_counter)])


    sim_specific_arguments_copy = copy.copy(sim_specific_arguments)
    sim_specific_arguments_copy.client = None

    cur_info = {
        # for analysis
        'x': x,
        'objectives': objectives,
        'labels': fuzzing_content.labels,

        'is_bug': is_bug,
        'bug_type': bug_type,

        'xl': np.array([pair[1] for pair in fuzzing_content.parameters_min_bounds.items()]),
        'xu': np.array([pair[1] for pair in fuzzing_content.parameters_max_bounds.items()]),
        'mask': fuzzing_content.mask,

        # for rerun
        'fuzzing_content': fuzzing_content,
        'fuzzing_arguments': fuzzing_arguments,
        'sim_specific_arguments': sim_specific_arguments_copy,
        'dt_arguments': dt_arguments,
        'counter': sim_counter,

        # diversity specific
        'trajectory_vector': trajectory_vector,
    }

    copy_tree(temporary_running_folder, cur_folder)
    with open(cur_folder+'/'+'cur_info.pickle', 'wb') as f_out:
        pickle.dump(cur_info, f_out)
    print('finish copy_tree')

    return objectives, cur_info


if __name__ == "__main__":
    from tools.sim.op_script.op_specific import initialize_op_specific
    from tools.sim.op_script.scene_configs import customized_bounds_and_distributions
    import atexit
    x = np.array([80, 60, 100, 1, 0, 15, 0, 0, 0, 0.2]+[1, 0, 45]+[2, 10, 0, 0, -50, 1, 0, -1, 0])

    dt_arguments  = None
    sim_counter = 0
    port = 2003
    launch_server = True

    atexit.register(exit_handler, [port])

    fuzzing_content = emptyobject(search_space_info=emptyobject(num_of_vehicle_behavior_changes=2, num_of_vehicles_max=1), labels=[], parameters_min_bounds={}, parameters_max_bounds={}, mask=[], customized_center_transforms=[])

    fuzzing_arguments = emptyobject(parent_folder='run_results_op', route_type='Town04_Opt_left_highway_entrance', objective_weights=np.array([1., 0., 0., 0., -1., -2., -1.]), default_objectives=np.array([130., 0., 0., 1., 0., 0., 0.]), carla_path=carla_root+'/CarlaUE4.sh', ego_car_model='op', traj_dist_metric='nearest')

    sim_specific_arguments = initialize_op_specific(fuzzing_arguments)


    run_op_simulation(x, fuzzing_content, fuzzing_arguments, sim_specific_arguments, dt_arguments, launch_server, sim_counter, port)

    sim_counter = 1
    launch_server  = False
    run_op_simulation(x, fuzzing_content, fuzzing_arguments, sim_specific_arguments, dt_arguments, launch_server, sim_counter, port)
