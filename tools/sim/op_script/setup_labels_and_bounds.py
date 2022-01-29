'''
CARLA Labels API
'''
from collections import OrderedDict
from tools.sim.op_script.object_types import vehicle_types


class emptyobject():
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


keywords_dict = {
    "num_of_vehicle_types": len(vehicle_types),
}
num_of_vehicle_behavior_changes = 5


weather_fields = [
    ("cloudiness", "real", 0, 100),
    ("precipitation", "real", 0, 80),
    ("precipitation_deposits", "real", 0, 80),
    ("wind_intensity", "real", 0, 50),
    ("sun_azimuth_angle", "real", 0, 360),
    ("sun_altitude_angle", "real", -90, 90),
    ("fog_density", "real", 0, 15),
    ("fog_distance", "real", 0, 100),
    ("wetness", "real", 0, 40),
    ("fog_falloff", "real", 0, 2),
]

general_fields = [
    ("num_of_vehicles", "int", 1, 4),
]



ego_car_fields = [
    ("delay_time_to_start", "real", 0, 0),
    ("ego_maximum_speed", "int", 25*1.6, 75*1.6)
]

vehicle_behavior_fields = [
    ("vehicle_lane_change", "int", 0, 2),
    ("vehicle_speed", "real", -100, 50),
]

vehicle_general_fields = [
    ("num_of_vehicle_types", "int", 0, len(vehicle_types)-1),
    ("vehicle_x", "real", -30, 30),
    ("vehicle_y", "real", -7, 3.5),
    ("vehicle_yaw", "real", 0, 0),
    ("vehicle_speed", "real", -100, 50),
]



def setup_bounds_mask_labels_distributions_stage1():
    fixed_hyperparameters = {
        "num_of_vehicle_types": len(vehicle_types),
        "num_of_vehicle_behavior_changes": num_of_vehicle_behavior_changes,
    }
    parameters_min_bounds = OrderedDict()
    parameters_max_bounds = OrderedDict()
    mask = []
    labels = []
    fields = weather_fields + general_fields + ego_car_fields

    for label, data_type, low, high in fields:
        labels.append(label)
        mask.append(data_type)
        parameters_min_bounds

        k_min = "_".join([label, "min"])
        k_max = "_".join([label, "max"])
        k = "_".join([label])

        parameters_min_bounds[k_min] = low
        parameters_max_bounds[k_max] = high

    return (
        fixed_hyperparameters,
        parameters_min_bounds,
        parameters_max_bounds,
        mask,
        labels,
    )


# Set up default bounds, mask, labels, and distributions for a Problem object
def setup_bounds_mask_labels_distributions_stage2(
    fixed_hyperparameters, parameters_min_bounds, parameters_max_bounds, mask, labels
):
    # vehicles
    for i in range(parameters_max_bounds["num_of_vehicles_max"]):
        for label, data_type, low, high in vehicle_general_fields:
            k_min = "_".join([label, "min", str(i)])
            k_max = "_".join([label, "max", str(i)])
            k = "_".join([label, str(i)])

            labels.append(k)
            mask.append(data_type)
            parameters_min_bounds[k_min] = low
            parameters_max_bounds[k_max] = high

        for p in range(fixed_hyperparameters["num_of_vehicle_behavior_changes"]):
            for label, data_type, low, high in vehicle_behavior_fields:
                k_min = "_".join(["vehicle", str(i), label, "min", str(p)])
                k_max = "_".join(["vehicle", str(i), label, "max", str(p)])
                k = "_".join(["vehicle", str(i), label, str(p)])

                labels.append(k)
                mask.append(data_type)
                parameters_min_bounds[k_min] = low
                parameters_max_bounds[k_max] = high



    parameters_distributions = OrderedDict()
    for label in labels:
        parameters_distributions[label] = "uniform"

    # count the total number of fields
    n_var = len(weather_fields + general_fields + ego_car_fields) + parameters_max_bounds["num_of_vehicles_max"]* (len(vehicle_general_fields) + fixed_hyperparameters["num_of_vehicle_behavior_changes"] * len(vehicle_behavior_fields))


    return (
        fixed_hyperparameters,
        parameters_min_bounds,
        parameters_max_bounds,
        mask,
        labels,
        parameters_distributions,
        n_var,
    )


def assign_key_value_pairs(search_space_info, fixed_hyperparameters, parameters_min_bounds, parameters_max_bounds):
    for d in [fixed_hyperparameters, parameters_min_bounds, parameters_max_bounds]:
        for k, v in d.items():
            assert not hasattr(search_space_info, k), k+'should not appear twice.'
            setattr(search_space_info, k, v)

# Customize parameters
def customize_parameters(parameters, customized_parameters):
    for k, v in customized_parameters.items():
        if k in parameters:
            parameters[k] = v
        else:
            # print(k, 'is not defined in the parameters.')
            pass

def generate_fuzzing_content(customized_config):
    customized_parameters_bounds = customized_config['customized_parameters_bounds']

    customized_parameters_distributions = customized_config['customized_parameters_distributions']

    customized_center_transforms = customized_config['customized_center_transforms']

    customized_constraints = customized_config['customized_constraints']

    fixed_hyperparameters, parameters_min_bounds, parameters_max_bounds, mask, labels = setup_bounds_mask_labels_distributions_stage1()
    customize_parameters(parameters_min_bounds, customized_parameters_bounds)
    customize_parameters(parameters_max_bounds, customized_parameters_bounds)

    fixed_hyperparameters, parameters_min_bounds, parameters_max_bounds, mask, labels, parameters_distributions, n_var = setup_bounds_mask_labels_distributions_stage2(fixed_hyperparameters, parameters_min_bounds, parameters_max_bounds, mask, labels)

    customize_parameters(parameters_min_bounds, customized_parameters_bounds)
    customize_parameters(parameters_max_bounds, customized_parameters_bounds)
    customize_parameters(parameters_distributions, customized_parameters_distributions)

    search_space_info = emptyobject()
    assign_key_value_pairs(search_space_info, fixed_hyperparameters, parameters_min_bounds, parameters_max_bounds)


    fuzzing_content = emptyobject(labels=labels, mask=mask, parameters_min_bounds=parameters_min_bounds, parameters_max_bounds=parameters_max_bounds, parameters_distributions=parameters_distributions, customized_constraints=customized_constraints, customized_center_transforms=customized_center_transforms, n_var=n_var, fixed_hyperparameters=fixed_hyperparameters,
    search_space_info=search_space_info, keywords_dict=keywords_dict)

    return fuzzing_content
