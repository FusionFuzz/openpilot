# Adapted Implemntation of MATLAB FCW Fusion logic
# https://www.mathworks.com/help/driving/ug/forward-collision-warning-using-sensor-fusion.html


import numpy as np
from scipy.linalg import block_diag
from collections import deque
from scipy.optimize import linear_sum_assignment
from selfdrive.controls.lib.radar_helpers import _LEAD_ACCEL_TAU
from filterpy.kalman import KalmanFilter
from common.numpy_fast import interp
from selfdrive.config import RADAR_TO_CAMERA
from copy import deepcopy
import cereal.messaging as messaging

lead_dict_default = {
  "dRel": 0,
  "yRel": 0,
  "vRel": 0,
  "vLead": 0,
  "vLeadK": 0,
  "aLeadK": 0,
  "fcw": False,
  "modelProb": 0,
  "radar": False,
  "aLeadTau": 0,
  "status": False
}

# definitions


# design choice: a different threshold than 35 used in MATLAB
assignment_threshold = 12
# delete a track if 5 out of 5 disappearance
deletion_threshold = [0, 5]
# confirm a track if 2 out of 3 appearance
confirmation_threshold = [2, 3]

appearance_cache_len = np.max([deletion_threshold[-1], confirmation_threshold[-1]])


# TBD: update parameters
def initialize_kalman(object):
    # x; vx; ax; y; vy; ay
    kf = KalmanFilter(dim_x=6, dim_z=3)

    x, vx, y = object.x, object.vx, object.y
    vy, ax, ay = 0, 0, 0
    kf.x = np.array([x, vx, ax, y, vy, ay])
    dt = 0.05
    # state transition matrix
    F = np.array([[1., dt, dt**2/2],
                     [0.,  1., dt],
                     [0, 0, 1]])
    kf.F = block_diag(F, F)

    # Measurement function
    kf.H = np.array([[1., 0., 0., 0., 0., 0.],
                     [0., 1., 0., 0., 0., 0.],
                     [0., 0., 0., 1., 0., 0.]])
    # measurement uncertainty
    # design choice: look into radar/camera spec
    R = np.array([[1, 0, 0],
                  [0, 0.3, 0],
                  [0, 0, 0.5]])
    kf.R[:] = R

    # design choice: state covariance (based on measurement noise)
    P1 = np.array([[1, 0, 0],
                   [0, 0.25, 0],
                   [0, 0, 100]])
    P2 = np.array([[0.1, 0, 0],
                   [0, 100, 0],
                   [0, 0, 100]])
    kf.P[:] = block_diag(P1, P2)

    # process noise
    sigma = 1 # Magnitude of the unknown acceleration change rate
    Q = np.array([[dt**4/4, dt**3/2, dt**2/2],
                  [dt**3/2, dt**2, dt],
                  [dt**2/2, dt, 1]])
    Q = block_diag(Q, Q)
    kf.Q[:] = Q

    return kf



# trackers
class Tracker:
    def __init__(self, object):
        # 'tentative', 'tracked', 'discarded'
        self.status = 'tentative'
        self.recent_appearances = deque([0 for _ in range(appearance_cache_len)])
        self.recent_appearances[-1] = 1
        self.kf = initialize_kalman(object)
        self.sensor = object.sensor
        self.modelProb = object.modelProb
        self.age = 0
        # design choice: this one has one more value to account for the case of cut in: 'in lane', 'moving', 'moving towards lane'
        self.condition = object.condition
        self.d_to_l = object.d_to_l

    def update(self, object, cost):
        if cost < assignment_threshold:
            detected = True
            self.recent_appearances.append(1)
        else:
            detected = False
            self.recent_appearances.append(0)
        if len(self.recent_appearances) > appearance_cache_len:
            self.recent_appearances.popleft()

        # confirm tracks
        if detected and self.status == 'tentative':
            if np.sum(list(self.recent_appearances)[-confirmation_threshold[1]:]) >= confirmation_threshold[0]:
                self.status = 'tracked'
        # delete lost tracks
        if not detected:
            # print('self.status prev', self.status)
            if self.status == 'tracked' and np.sum(list(self.recent_appearances)[-deletion_threshold[1]:]) <= deletion_threshold[0]:
                self.status = 'discarded'
            elif self.status == 'tentative' and np.sum(list(self.recent_appearances)[-confirmation_threshold[1]:]) <= confirmation_threshold[0] and self.age >=  confirmation_threshold[1]:
                self.status = 'discarded'
            # print('self.recent_appearances', self.recent_appearances)
            # print('self.status after', self.status)

        # update assigned tracks
        if self.status in ['tentative', 'tracked']:
            self.kf.predict()
            if object is not None:
                self.kf.update(np.array([object.x, object.vx, object.y]))

                if self.sensor != object.sensor:
                    if self.sensor:
                        print('sensor update:', self.sensor, '-->', object.sensor)
                    self.sensor = object.sensor

                # design choice: check if the tracked object is cutting in
                if self.condition in ['moving', 'moving towards lane'] and object.condition == 'moving':
                    if object.d_to_l < self.d_to_l:
                        self.condition = 'moving towards lane'
                    else:
                        self.condition = 'moving'
                else:
                    self.condition = object.condition

            else:
                self.kf.update(None)


        self.age += 1


class object_data:
    def __init__(self, x=0, vx=0, y=0, sensor=None, modelProb=0):
        self.x = x
        self.vx = vx
        self.y = y
        # "radar", "camera"
        self.sensor = sensor
        self.modelProb = modelProb
        # self.measurement_noise = measurement_noise
        # self.object_attributes = object_attributes

        # design choice: condition will be used for radar object. It is mainly used to filter out moving objects on another lane that is driving in parallel
        # 'in lane', 'moving'
        self.condition = None

        # design choice: object's distance to estimated corresponding ego car's left/right lanes y values when the tracked object's x is used
        self.d_to_l = 9999


# return the closest horizontal distance between (x, y) and the predicted ego car's lanes
def d_to_closest_lane(x, y, lanes):
    obj_lane_l = interp(x, lanes[1].x, lanes[1].y)
    obj_lane_r = interp(x, lanes[2].x, lanes[2].y)
    if -y < obj_lane_l:
        return np.abs(obj_lane_l - (-y))
    elif -y > obj_lane_r:
        return np.abs(-y - obj_lane_r)
    else:
        return 0

# helpers
def filter_non_clutter_objects_radar(clusters, lanes):
    objects_radar = []
    print('\n'*2)
    print('--filter_non_clutter_objects_radar--')

    for cluster in clusters:
        # TBD: may not want to set radar model prob to 1 in any cases
        # TBD: the sign of y is flipped
        obj_r = object_data(cluster.dRel, cluster.vLead, cluster.yRel, 'radar', 1)

        # design choice: check if obj_r in lanes
        d_to_l = d_to_closest_lane(obj_r.x, obj_r.y, lanes)
        obj_r.d_to_l = d_to_l
        # hack: camera/radar y are flipped but lane lines y are not
        if d_to_l == 0:
            obj_r.condition = 'in lane'
            objects_radar.append(obj_r)
            print('in lane: obj_r.x, obj_r.y, obj_r.vx', obj_r.x, obj_r.y, obj_r.vx)
        # design choice: may need to change threshold
        elif np.abs(obj_r.vx) > 1:
            print('moving: obj_r.x, obj_r.y, obj_r.vx', obj_r.x, obj_r.y, obj_r.vx)
            obj_r.condition = 'moving'
            objects_radar.append(obj_r)
        else:
            print('clutter: obj_r.x, obj_r.y, obj_r.vx', obj_r.x, obj_r.y, obj_r.vx)
    print('-'*20)
    print('\n'*2)

    return objects_radar


def reformat_objects_camera(lead_msg_list, v_ego, lanes):
    print('\n'*2)
    print('++reformat_objects_camera++')
    objects_camera = []

    # # design choice: remove repetitions in lead_msg_list
    # if len(lead_msg_list) == 2:
    #     same = True
    #     lead_msg1 = lead_msg_list[0]
    #     lead_msg2 = lead_msg_list[1]
    #     for i in range(4):
    #         if (lead_msg1.xyva[i] - lead_msg2.xyva[i]) > 1e-3:
    #             same = False
    #             break
    #     if same:
    #         lead_msg_list = [lead_msg_list[0]]

    for lead_msg in lead_msg_list:
        x1, y1, vx1, modelProb = lead_msg.xyva[0], lead_msg.xyva[1], lead_msg.xyva[2], lead_msg.prob
        # design choice: only when model prob > 0.5 the camera detected object is considered
        if modelProb > 0.5:
            obj_c_1 = object_data(x1 - RADAR_TO_CAMERA, vx1+v_ego, -y1, 'camera', modelProb)

            d_to_l = d_to_closest_lane(obj_c_1.x, obj_c_1.y, lanes)
            obj_c_1.d_to_l = d_to_l
            # all camera objects are in lane
            obj_c_1.condition = 'in lane'

            objects_camera.append(obj_c_1)
            print('camera: x1, y1, vx1, modelProb', obj_c_1.x, obj_c_1.y, obj_c_1.vx, obj_c_1.modelProb)
    print('+'*20)
    print('\n'*2)

    return objects_camera

def merge_radar_and_camera(objects_radar, objects_camera):
    objects_all = objects_radar + objects_camera
    return objects_all

def find_most_important_objects(trackers, lanes, v_ego, model_output_frameId, num=2, fusion='mathwork_in_lane', pm=None):
    # TBD: get in lane objects and pick the closest num of objects
    def sort_by_ttc(t):
        if v_ego > (t.kf.x[1]+1):
            return t.kf.x[0]/(v_ego-t.kf.x[1])
        else:
            return t.kf.x[0]
    # print('lanes[1].x, lanes[1].y', lanes[1].x, lanes[1].y)
    # print('lanes[2].x, lanes[2].y', lanes[2].x, lanes[2].y)
    lead_trackers_dict = [lead_dict_default for _ in range(num)]
    candidate_trackers = []
    if len(trackers) > 0:
        for tracker in trackers:
            d_to_l = d_to_closest_lane(tracker.kf.x[0], tracker.kf.x[3], lanes)
            # hack: camera/radar y are flipped but lane lines y are not
            if fusion == 'mathwork_all':
                candidate_trackers.append(tracker)
            elif fusion == 'mathwork_in_lane':
                if d_to_l == 0:
                    print('in line')
                    candidate_trackers.append(tracker)
            elif fusion == 'mathwork_moving':
                if d_to_l == 0:
                    print('in line')
                    candidate_trackers.append(tracker)
                elif tracker.condition == 'moving towards lane':
                    print('\n'*3, 'moving towards lane', '\n'*3)
                    candidate_trackers.append(tracker)
            else:
                raise Exception('unknow fusion: '+fusion)
        # sort by smoothed ttc: x/(v_ego-vx)
        trackers_sorted_list = sorted(candidate_trackers, key=sort_by_ttc)

        trackers_sorted_dict_list = reformat_to_lead_dict(trackers_sorted_list, v_ego)


        replace_n = np.min([num, len(trackers_sorted_dict_list)])
        lead_trackers_dict[:replace_n] = trackers_sorted_dict_list[:replace_n]


        if pm is not None:
            lead_radar_selected = 0
            lead_vision_selected = 0
            lead_dict_vision_1 = lead_dict_default
            lead_dict_vision_2 = deepcopy(lead_dict_default)
            lead_dict_radar_1 = deepcopy(lead_dict_default)
            lead_dict_radar_2 = deepcopy(lead_dict_default)
            for tracker_dict in trackers_sorted_dict_list:
                if tracker_dict['radar'] == True:
                    if lead_radar_selected == 0:
                        lead_dict_radar_1 = tracker_dict
                        lead_radar_selected += 1
                    elif lead_radar_selected == 1:
                        lead_dict_radar_2 = tracker_dict
                        lead_radar_selected += 1
                elif tracker_dict['radar'] == False:
                    if lead_vision_selected == 0:
                        lead_dict_vision_1 = tracker_dict
                        lead_vision_selected += 1
                    elif lead_vision_selected == 1:
                        lead_dict_vision_2 = tracker_dict
                        lead_vision_selected += 1
                elif lead_radar_selected == 2 and lead_vision_selected == 2:
                    break

            dat_leads = messaging.new_message('radardData')
            dat_leads.radardData = {
              'frameId': model_output_frameId,
              'leadRadar1': lead_dict_radar_1,
              'leadVision1': lead_dict_vision_1,
              'leadRadar2': lead_dict_radar_2,
              'leadVision2': lead_dict_vision_2,
              'leadPredicted1': lead_trackers_dict[0],
              'leadPredicted2': lead_trackers_dict[1]
            }
            pm.send('radardData', dat_leads)




    return lead_trackers_dict

def reformat_to_lead_dict(lead_tracks, v_ego):
    lead_dicts_list = []
    for lead_track in lead_tracks:
        if lead_track.sensor == 'radar':
            radar = True
        else:
            radar = False

        v_k, a_k = np.dot(lead_track.kf.F, lead_track.kf.x)[1:3]
        x, y, vx = lead_track.kf.x[0], lead_track.kf.x[3], lead_track.kf.x[1]
        lead_dict = {
          "dRel": float(x),
          "yRel": float(y),
          "vRel": float(vx - v_ego),
          "vLead": float(vx),
          "vLeadK": float(v_k),
          # might need to update if radar tracks are not restarted every time
          "aLeadK": float(a_k),

          # might need to update if radar tracks are not restarted every time
          "aLeadTau": _LEAD_ACCEL_TAU,
          "fcw": True,
          "modelProb": float(lead_track.modelProb),
          "radar": radar,
          "status": True
        }
        lead_dicts_list.append(lead_dict)
    return lead_dicts_list

def get_lead2(v_ego, ready, clusters, lead_msg_list, model_output_frameId, lanes, trackers, fusion='mathwork_in_lane', pm=None):
    # lanes = sm['modelV2'].laneLines
    # preprocessing radar (keep non-cluttered objects)
    objects_radar = filter_non_clutter_objects_radar(clusters, lanes)

    if ready:
        # preprocessing camera
        objects_camera = reformat_objects_camera(lead_msg_list, v_ego, lanes)
        # merge radar and camera
        objects_all = merge_radar_and_camera(objects_radar, objects_camera)
    else:
        objects_all = objects_radar


    # apply kalman filter to detection
    # row: detections, col: tracks
    detections_num = len(objects_all)
    tracks_num = len(trackers)
    cost_matrix = np.zeros([detections_num, tracks_num])
    print('detections_num, tracks_num', detections_num, tracks_num)
    detection_inds = np.arange(detections_num)
    tracks_inds = np.arange(tracks_num)
    row_inds, col_inds = np.array([]), np.array([])

    # (0) estimate cost matrix
    if len(trackers) > 0 and len(objects_all) > 0:
        for i, object in enumerate(objects_all):
            for j, tracker in enumerate(trackers):
                det_x, det_vx, det_y = object.x, object.vx, object.y
                kf_x, kf_vx, kf_y = tracker.kf.x[0], tracker.kf.x[1], tracker.kf.x[3]
                # TBD: update parameters
                dx = (kf_x - det_x)
                dvx = (kf_vx - det_vx) * 2
                dy = (kf_y - det_y) * 5
                cost = dx**2 + dvx**2 + dy**2
                cost_matrix[i, j] = cost
        print('cost_matrix', cost_matrix)
        # (1) match detections with tracks: hungarian matching
        row_inds, col_inds = linear_sum_assignment(cost_matrix)

        # (2) update assigned tracks / confirm tracks
        for ri, ci in zip(row_inds, col_inds):
            trackers[ci].update(objects_all[ri], cost_matrix[ri, ci])

    # (3) update unssigned track
    if len(trackers) > 0:
        unassigned_track_inds = np.setdiff1d(tracks_inds, col_inds)
        print('unassigned_track_inds', unassigned_track_inds)
        for ci in unassigned_track_inds:
            print(ci)
            trackers[ci].update(None, assignment_threshold+1)

    # (4) create new tracks
    if len(objects_all) > 0:
        unassigned_detection_inds = np.setdiff1d(detection_inds, row_inds)
        for ri in unassigned_detection_inds:
            trackers.append(Tracker(objects_all[ri]))

    # (5) delete lost tracks
    new_trackers = []
    lead_dicts_list = []
    if len(trackers) > 0:
        for tracker in trackers:
            if tracker.status != 'discarded':
                new_trackers.append(tracker)

    num = 2
    lead_trackers_dicts_list = find_most_important_objects(new_trackers, lanes, v_ego, model_output_frameId, num, fusion=fusion, pm=pm)
    print('lead_trackers_dicts_list', lead_trackers_dicts_list)
    return lead_trackers_dicts_list, new_trackers
