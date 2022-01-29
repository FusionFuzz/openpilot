#!/usr/bin/env python3
import importlib
import math
from collections import defaultdict, deque

import cereal.messaging as messaging
from cereal import car
from common.numpy_fast import interp
from common.params import Params
from common.realtime import Ratekeeper, Priority, config_realtime_process
from selfdrive.config import RADAR_TO_CAMERA
from selfdrive.controls.lib.cluster.fastcluster_py import cluster_points_centroid
from selfdrive.controls.lib.radar_helpers import Cluster, Track
from selfdrive.swaglog import cloudlog
from selfdrive.hardware import TICI

# addition:
import cereal.messaging as messaging
import sys

sys.path.append('../../tools/sim')

from tools.sim.op_script.utils_multiple import get_camera_lead_msg, reformat_radar_clusters
from tools.sim.op_script.op_specific import get_inds_with_errors, convert_lead_carla_to_dict
from copy import deepcopy

class KalmanParams():
  def __init__(self, dt):
    # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
    # hardcoding a lookup table to compute K for values of radar_ts between 0.1s and 1.0s
    assert dt > .01 and dt < .1, "Radar time step must be between .01s and 0.1s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    #Q = np.matrix([[10., 0.0], [0.0, 100.]])
    #R = 1e3
    #K = np.matrix([[ 0.05705578], [ 0.03073241]])
    dts = [dt * 0.01 for dt in range(1, 11)]
    K0 = [0.12288, 0.14557, 0.16523, 0.18282, 0.19887, 0.21372, 0.22761, 0.24069, 0.2531, 0.26491]
    K1 = [0.29666, 0.29331, 0.29043, 0.28787, 0.28555, 0.28342, 0.28144, 0.27958, 0.27783, 0.27617]
    self.K = [[interp(dt, dts, K0)], [interp(dt, dts, K1)]]


def laplacian_cdf(x, mu, b):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)

# modification
def match_vision_to_cluster(v_ego, lead, clusters, fusion='op'):
  # match vision point to best statistical cluster match
  offset_vision_dist = lead.xyva[0] - RADAR_TO_CAMERA

  def prob(c):
    prob_d = laplacian_cdf(c.dRel, offset_vision_dist, lead.xyvaStd[0])
    prob_y = laplacian_cdf(c.yRel, -lead.xyva[1], lead.xyvaStd[1])
    prob_v = laplacian_cdf(c.vRel, lead.xyva[2], lead.xyvaStd[2])

    # This is isn't exactly right, but good heuristic
    return prob_d * prob_y * prob_v

  cluster = max(clusters, key=prob)

  # if no 'sane' match is found return -1
  # stationary radar points can be false positives
  dist_sane = abs(cluster.dRel - offset_vision_dist) < max([(offset_vision_dist)*.25, 5.0])
  vel_sane = (abs(cluster.vRel - lead.xyva[2]) < 10) or (v_ego + cluster.vRel > 3)
  if dist_sane and vel_sane:
    return cluster
  else:
    # modification:
    if fusion == 'op':
        return None
    elif fusion == 'op_radar':
        return cluster
    else:
        raise Exception('unknow fusion: '+fusion)



# def get_lead(v_ego, ready, clusters, lead_msg, low_speed_override=True):
#   # Determine leads, this is where the essential logic happens
#   if len(clusters) > 0 and ready and lead_msg.prob > .5:
#     cluster = match_vision_to_cluster(v_ego, lead_msg, clusters)
#   else:
#     cluster = None
#
#   lead_dict = {'status': False}
#   if cluster is not None:
#     lead_dict = cluster.get_RadarState(lead_msg.prob)
#   elif (cluster is None) and ready and (lead_msg.prob > .5):
#     lead_dict = Cluster().get_RadarState_from_vision(lead_msg, v_ego)
#
#   if low_speed_override:
#     low_speed_clusters = [c for c in clusters if c.potential_low_speed_lead(v_ego)]
#     if len(low_speed_clusters) > 0:
#       closest_cluster = min(low_speed_clusters, key=lambda c: c.dRel)
#
#       # Only choose new cluster if it is actually closer than the previous one
#       if (not lead_dict['status']) or (closest_cluster.dRel < lead_dict['dRel']):
#         lead_dict = closest_cluster.get_RadarState()
#
#   return lead_dict


# addition:
def get_lead(v_ego, ready, clusters, lead_msg, lead_num, low_speed_override=True, fusion='op'):
  # addition:
  from selfdrive.controls.customized_get_lead import lead_dict_default

  # Determine leads, this is where the essential logic happens
  # addition
  # mode = 0
  # print('len(clusters)', len(clusters))
  # modification:
  if fusion == 'op':
      condition = (len(clusters) > 0) & ready & (lead_msg.prob > .5)
  elif fusion == 'op_radar':
      condition = (len(clusters) > 0) & ready
  else:
      raise Exception('unknow fusion: '+fusion)

  if condition:
    cluster = match_vision_to_cluster(v_ego, lead_msg, clusters, fusion=fusion)
    # print('ready', ready, 'lead_msg.prob', lead_msg.prob)
    # mode = 1
  else:
    cluster = None

  lead_dict = lead_dict_default



  lead_dict_radar = lead_dict_default
  lead_dict_vision = deepcopy(lead_dict_default)

  if cluster is not None:
    lead_dict_radar = cluster.get_RadarState(lead_msg.prob)
    lead_dict = deepcopy(lead_dict_radar)
    # mode = 2
  if ready and (lead_msg.prob > .5):
    lead_dict_vision = Cluster().get_RadarState_from_vision(lead_msg, v_ego)
    if cluster is None:
        lead_dict = deepcopy(lead_dict_vision)
        # mode = 3

  if low_speed_override:
    low_speed_clusters = [c for c in clusters if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_clusters) > 0:
      closest_cluster = min(low_speed_clusters, key=lambda c: c.dRel)

      # Only choose new cluster if it is actually closer than the previous one
      if (not lead_dict['status']) or (closest_cluster.dRel < lead_dict['dRel']):
        lead_dict_radar = closest_cluster.get_RadarState()
        lead_dict = deepcopy(lead_dict_radar)
        # mode = 4


  # modification:
  return lead_dict, lead_dict_radar, lead_dict_vision


class RadarD():
  # modification
  def __init__(self, radar_ts, delay=0, pm=None):
    self.current_time = 0

    self.tracks = defaultdict(dict)
    self.kalman_params = KalmanParams(radar_ts)

    # v_ego
    self.v_ego = 0.
    self.v_ego_hist = deque([0], maxlen=delay+1)

    self.ready = False

    # addition
    self.pm = pm
    self.trackers = []
    self.fusion = 'op'

  def update(self, sm, rr, enable_lead):
    # addition:
    from selfdrive.controls.customized_get_lead import lead_dict_default

    self.current_time = 1e-9*max(sm.logMonoTime.values())

    if sm.updated['carState']:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
    if sm.updated['modelV2']:
      self.ready = True

    # addition:
    if sm.updated['parametersFromCarla']:
        cur_fusion = sm['parametersFromCarla'].fusion
        if self.fusion != cur_fusion:
            self.fusion = cur_fusion



    ar_pts = {}
    for pt in rr.points:
      ar_pts[pt.trackId] = [pt.dRel, pt.yRel, pt.vRel, pt.measured]
      # print('pt.trackId, pt.dRel, pt.yRel, pt.vRel, pt.measured', pt.trackId, pt.dRel, pt.yRel, pt.vRel, pt.measured)
      # addition
      # print('pt.trackId, pt.dRel, pt.yRel, pt.vRel, pt.measured', pt.trackId, pt.dRel, pt.yRel, pt.vRel, pt.measured)

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # *** compute the tracks ***
    for ids in ar_pts:
      rpt = ar_pts[ids]
      # print('ids, rpt', ids, rpt)
      # align v_ego by a fixed time to align it with the radar measurement
      v_lead = rpt[2] + self.v_ego_hist[0]

      # create the track if it doesn't exist or it's a new track
      if ids not in self.tracks:
        self.tracks[ids] = Track(v_lead, self.kalman_params)
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, rpt[3])

    idens = list(sorted(self.tracks.keys()))
    track_pts = list([self.tracks[iden].get_key_for_cluster() for iden in idens])

    # If we have multiple points, cluster them
    if len(track_pts) > 1:
      cluster_idxs = cluster_points_centroid(track_pts, 2.5)
      # addition:
      # print('track_pts', track_pts)
      clusters = [None] * (max(cluster_idxs) + 1)

      for idx in range(len(track_pts)):
        cluster_i = cluster_idxs[idx]
        if clusters[cluster_i] is None:
          clusters[cluster_i] = Cluster()
        clusters[cluster_i].add(self.tracks[idens[idx]])

    elif len(track_pts) == 1:
      # FIXME: cluster_point_centroid hangs forever if len(track_pts) == 1
      cluster_idxs = [0]
      clusters = [Cluster()]
      clusters[0].add(self.tracks[idens[0]])
    else:
      clusters = []

    # if a new point, reset accel to the rest of the cluster
    for idx in range(len(track_pts)):
      if self.tracks[idens[idx]].cnt <= 1:
        aLeadK = clusters[cluster_idxs[idx]].aLeadK
        aLeadTau = clusters[cluster_idxs[idx]].aLeadTau
        self.tracks[idens[idx]].reset_a_lead(aLeadK, aLeadTau)

    # *** publish radarState ***
    dat = messaging.new_message('radarState')
    dat.valid = sm.all_alive_and_valid() and len(rr.errors) == 0
    radarState = dat.radarState
    radarState.mdMonoTime = sm.logMonoTime['modelV2']
    radarState.canMonoTimes = list(rr.canMonoTimes)
    radarState.radarErrors = list(rr.errors)
    radarState.carStateMonoTime = sm.logMonoTime['carState']


    # addition:
    lead_dict_selected_1 = lead_dict_default
    lead_dict_selected_2 = lead_dict_default
    model_output = sm['modelV2']
    lead_carla = sm['leadCarla']


    # addition: publish clusters
    dat_radar_clusters = messaging.new_message('radarClusters', len(clusters))
    for cnt, cluster in enumerate(clusters):
        dat_radar_clusters.radarClusters[cnt] = {
            "dRel": float(cluster.dRel),
            "yRel": float(cluster.yRel),
            "vRel": float(cluster.vRel),
            "aRel": float(0.),
            "frameId": model_output.frameId
        }
    self.pm.send('radarClusters', dat_radar_clusters)


    # modification
    if 'op' in self.fusion:
        if enable_lead:
          if len(model_output.leads) > 1:
            # modifications
            lead_dict_selected_1, lead_dict_radar_1, lead_dict_vision_1 = get_lead(self.v_ego, self.ready, clusters, model_output.leads[0], 0, low_speed_override=True, fusion=self.fusion)
            # modification
            lead_dict_selected_2, lead_dict_radar_2, lead_dict_vision_2 = get_lead(self.v_ego, self.ready, clusters, model_output.leads[1], 1, low_speed_override=False, fusion=self.fusion)

            dat_leads = messaging.new_message('radardData')
            dat_leads.radardData = {
              'frameId': model_output.frameId,
              'leadRadar1': lead_dict_radar_1,
              'leadVision1': lead_dict_vision_1,
              'leadRadar2': lead_dict_radar_2,
              'leadVision2': lead_dict_vision_2,
              'leadPredicted1': lead_dict_selected_1,
              'leadPredicted2': lead_dict_selected_2
            }
            self.pm.send('radardData', dat_leads)
    # addition
    elif 'mathwork' in self.fusion:
        from selfdrive.controls.customized_get_lead import get_lead2
        if enable_lead:
            if len(model_output.leads) > 1:
                lead_msg_list = [model_output.leads[0], model_output.leads[1]]
                lead_dicts_list, self.trackers = get_lead2(self.v_ego, self.ready, clusters, lead_msg_list,  model_output.frameId, model_output.laneLines, self.trackers, fusion=self.fusion, pm=self.pm)
                if len(lead_dicts_list) == 0:
                    lead_dict_selected_1, lead_dict_selected_2 = lead_dict_default, lead_dict_default
                elif len(lead_dicts_list) == 1:
                    lead_dict_selected_1, lead_dict_selected_2 = lead_dicts_list[0], lead_dict_default
                else:
                    lead_dict_selected_1, lead_dict_selected_2 = lead_dicts_list[0], lead_dicts_list[1]
    elif 'best_sensor' in self.fusion:
        camera_leads = get_camera_lead_msg(model_output)
        radar_clusters_reformated = reformat_radar_clusters(clusters)

        lead_carla = convert_lead_carla_to_dict(lead_carla)
        error_type_ind_list_c = get_inds_with_errors(camera_leads, lead_carla, 'camera')
        error_type_ind_list_r = get_inds_with_errors(radar_clusters_reformated, lead_carla, 'radar')

        sorted_error_type_ind_list = sorted(error_type_ind_list_c + error_type_ind_list_r, key=lambda t:t[0])


        lead_dicts = [lead_dict_default, lead_dict_default]
        for i, (_, type, ind) in enumerate(sorted_error_type_ind_list[:min([2, len(sorted_error_type_ind_list)])]):
            if type == 'camera':
                lead_dict_selected_i = Cluster().get_RadarState_from_vision(model_output.leads[ind], self.v_ego)
            elif type == 'radar':
                lead_dict_selected_i = clusters[ind].get_RadarState(1)
            else:
                raise
            lead_dicts[i] = lead_dict_selected_i

        lead_dict_selected_1, lead_dict_selected_2 = lead_dicts



        # if ind1 >= 0 and ind2 >=0:
        #     print('error1', error1, 'error2', error2)
        #     print('lead_camera', Cluster().get_RadarState_from_vision(model_output.leads[ind1], self.v_ego))
        #     print('lead_radar', clusters[ind2].get_RadarState(1))
        #     print('lead_carla', lead_carla)

        # if ind1 >= 0 or ind2 >=0:
        #     if error1 < error2:
        #         lead_dict_selected_1 = Cluster().get_RadarState_from_vision(model_output.leads[ind1], self.v_ego)
        #     else:
        #         if ind1 >= 0:
        #             prob = camera_leads[ind1]['modelProb']
        #         else:
        #             prob = 1
        #         lead_dict_selected_1 = clusters[ind2].get_RadarState(prob)
        # lead_dict_selected_2 = deepcopy(lead_dict_selected_1)

        dat_leads = messaging.new_message('radardData')
        dat_leads.radardData = {
          'frameId': model_output.frameId,
          'leadRadar1': lead_dict_default,
          'leadVision1': lead_dict_default,
          'leadRadar2': lead_dict_default,
          'leadVision2': lead_dict_default,
          'leadPredicted1': lead_dict_selected_1,
          'leadPredicted2': lead_dict_selected_2
        }
        self.pm.send('radardData', dat_leads)

    elif 'ground_truth' in self.fusion:
        pass
    else:
        raise Exception('unknow fusion: '+self.fusion)

    # addition:
    if lead_dict_selected_1 is not None and lead_dict_selected_2 is not None:
        radarState.leadOne = lead_dict_selected_1
        radarState.leadTwo = lead_dict_selected_2


    return dat


# fuses camera and radar data for best lead detection
# modification
def radard_thread(sm=None, pm=None, can_sock=None):
  config_realtime_process(5 if TICI else 2, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = car.CarParams.from_bytes(Params().get("CarParams", block=True))
  cloudlog.info("radard got CarParams")

  # import the radar from the fingerprint
  cloudlog.info("radard is importing %s", CP.carName)
  RadarInterface = importlib.import_module('selfdrive.car.%s.radar_interface' % CP.carName).RadarInterface

  # *** setup messaging
  if can_sock is None:
    can_sock = messaging.sub_sock('can')
  if sm is None:
    # modification
    sm = messaging.SubMaster(['modelV2', 'carState', 'leadCarla', 'parametersFromCarla'], ignore_avg_freq=['modelV2', 'carState', 'leadCarla', 'parametersFromCarla'])  # Can't check average frequency, since radar determines timing
  if pm is None:
    # modification
    pm = messaging.PubMaster(['radarState', 'liveTracks', 'radardData', 'radarClusters'])

  RI = RadarInterface(CP)

  rk = Ratekeeper(1.0 / CP.radarTimeStep, print_delay_threshold=None)

  RD = RadarD(CP.radarTimeStep, RI.delay, pm)

  # TODO: always log leads once we can hide them conditionally
  enable_lead = CP.openpilotLongitudinalControl or not CP.radarOffCan

  while 1:
    can_strings = messaging.drain_sock_raw(can_sock, wait_for_one=True)
    rr = RI.update(can_strings)

    if rr is None:
      continue

    sm.update(0)

    dat = RD.update(sm, rr, enable_lead)
    dat.radarState.cumLagMs = -rk.remaining*1000.

    pm.send('radarState', dat)

    # *** publish tracks for UI debugging (keep last) ***
    tracks = RD.tracks
    dat = messaging.new_message('liveTracks', len(tracks))

    for cnt, ids in enumerate(sorted(tracks.keys())):
      dat.liveTracks[cnt] = {
        "trackId": ids,
        "dRel": float(tracks[ids].dRel),
        "yRel": float(tracks[ids].yRel),
        "vRel": float(tracks[ids].vRel),
      }
    pm.send('liveTracks', dat)

    rk.monitor_time()

# modification
def main(sm=None, pm=None, can_sock=None):
  radard_thread(sm, pm, can_sock)


if __name__ == "__main__":
  main()
