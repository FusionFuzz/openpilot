CONTAINER_ID=$(docker ps | grep 'openpilot_client' | awk '{ print $1 }')
echo $CONTAINER_ID


# copy from local to docker
docker cp openpilot/tools/sim/lib/can.py $CONTAINER_ID:/openpilot/tools/sim/lib/can.py


docker cp -r ./op_script $CONTAINER_ID:/openpilot/tools/sim/



docker cp openpilot/selfdrive/controls/lib/fcw.py $CONTAINER_ID:/openpilot/selfdrive/controls/lib/fcw.py
docker cp openpilot/selfdrive/controls/lib/longitudinal_planner.py $CONTAINER_ID:/openpilot/selfdrive/controls/lib/longitudinal_planner.py


docker cp openpilot/selfdrive/controls/customized_get_lead.py $CONTAINER_ID:/openpilot/selfdrive/controls/customized_get_lead.py


docker cp openpilot/selfdrive/modeld/runners/onnx_runner.py $CONTAINER_ID:/openpilot/selfdrive/modeld/runners/onnx_runner.py


docker cp openpilot/cereal/services.py $CONTAINER_ID:/openpilot/cereal/services.py
docker cp openpilot/cereal/log.capnp $CONTAINER_ID:/openpilot/cereal/log.capnp


docker cp openpilot/selfdrive/controls/radard.py $CONTAINER_ID:/openpilot/selfdrive/controls/radard.py
docker cp openpilot/selfdrive/controls/lib/longcontrol.py $CONTAINER_ID:/openpilot/selfdrive/controls/lib/longcontrol.py
docker cp openpilot/selfdrive/controls/controlsd.py $CONTAINER_ID:/openpilot/selfdrive/controls/controlsd.py
docker cp openpilot/cereal/messaging/__init__.py $CONTAINER_ID:/openpilot/cereal/messaging/__init__.py
docker cp openpilot/selfdrive/manager/manager.py $CONTAINER_ID:/openpilot/selfdrive/manager/manager.py
docker cp openpilot/common/realtime.py $CONTAINER_ID:/openpilot/common/realtime.py
docker cp openpilot/selfdrive/manager/process_config.py $CONTAINER_ID:/openpilot/selfdrive/manager/process_config.py
docker cp openpilot/selfdrive/car/car_helper.py $CONTAINER_ID:/openpilot/selfdrive/car/car_helper.py
docker cp openpilot/selfdrive/modeld/modeld.cc $CONTAINER_ID:/openpilot/selfdrive/modeld/modeld.cc
docker cp openpilot/selfdrive/locationed/calibration.py $CONTAINER_ID:/openpilot/selfdrive/locationed/calibration.py


docker cp openpilot/cereal/visionipic/visionipc_server.cc $CONTAINER_ID:/openpilot/cereal/visionipic/visionipc_server.cc





# docker cp ../openpilot $CONTAINER_ID:/openpilot


# docker cp ../openpilot/selfdrive/manager/manager.py $CONTAINER_ID:/openpilot/selfdrive/manager/manager.py

# copy from docker to local
# docker cp $CONTAINER_ID:/openpilot/tools/sim/data_folder_motocycle ./
# docker cp $CONTAINER_ID:/openpilot/tools/sim/data_folder_vehicle ./
# docker cp $CONTAINER_ID:/openpilot/tools/sim/data_folder_mot_vehicle ./
# docker cp $CONTAINER_ID:/openpilot ./

# docker cp $CONTAINER_ID:/openpilot/tools/sim/motocycle ./motocycle
# docker cp $CONTAINER_ID:/openpilot/tools/sim/vehicle ./vehicle


docker cp $CONTAINER_ID:/openpilot/tools/sim/data_folder ./
