import cereal.messaging as messaging
from common.realtime import DT_DMON, DT_CTRL
from tools.sim.lib.can import can_function
import time

def panda_state_function(end_q=None):
  pm = messaging.PubMaster(['pandaState'])
  while 1:
    dat = messaging.new_message('pandaState')
    dat.valid = True
    dat.pandaState = {
      'ignitionLine': True,
      'pandaType': "blackPanda",
      'controlsAllowed': True,
      'safetyModel': 'hondaNidec'
    }
    pm.send('pandaState', dat)
    time.sleep(0.5)

    if not end_q.empty():
        return

def fake_driver_monitoring(end_q=None):
  pm = messaging.PubMaster(['driverState','driverMonitoringState'])
  while 1:

    # dmonitoringmodeld output
    dat = messaging.new_message('driverState')
    dat.driverState.faceProb = 1.0
    pm.send('driverState', dat)

    # dmonitoringd output
    dat = messaging.new_message('driverMonitoringState')
    dat.driverMonitoringState = {
      "faceDetected": True,
      "isDistracted": False,
      "awarenessStatus": 1.,
    }
    pm.send('driverMonitoringState', dat)

    time.sleep(DT_DMON)

    if not end_q.empty():
        return

def can_function_runner(vs, pm):
  i = 0
  while 1:
    # modification:
    can_function(pm, vs.speed, vs.angle, i, vs.cruise_button, vs.is_engaged, vs.radar_data)
    time.sleep(0.01)
    i+=1


# def send_panda_state_function():
#     pm = messaging.PubMaster(['pandaState'])
#
#     dat = messaging.new_message('pandaState')
#     dat.valid = True
#     dat.pandaState = {
#       'ignitionLine': True,
#       'pandaType': "blackPanda",
#       'controlsAllowed': True,
#       'safetyModel': 'hondaNidec'
#     }
#     pm.send('pandaState', dat)
#
#
# def send_fake_driver_monitoring():
#     pm = messaging.PubMaster(['driverState','driverMonitoringState'])
#     # dmonitoringmodeld output
#     dat = messaging.new_message('driverState')
#     dat.driverState.faceProb = 1.0
#     pm.send('driverState', dat)
#
#     # dmonitoringd output
#     dat = messaging.new_message('driverMonitoringState')
#     dat.driverMonitoringState = {
#       "faceDetected": True,
#       "isDistracted": False,
#       "awarenessStatus": 1.,
#     }
#     pm.send('driverMonitoringState', dat)
