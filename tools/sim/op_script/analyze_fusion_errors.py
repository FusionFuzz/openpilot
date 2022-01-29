from tools.sim.op_script.utils_multiple import append_relevant_paths
if __name__ == "__main__":
    append_relevant_paths()


import sys
import socket
import argparse
import os
import pickle
import numpy as np
import seaborn as sns
sns.set_theme()
sns.set_style("white")
import matplotlib.pyplot as plt
from tools.sim.op_script.op_specific import get_lead_interseced_keys, record_lead_and_error



'''
|-parent_folder
|---original
|---rerun_2.5_best_sensor
'''







from numpy import std, mean, sqrt
import itertools as it
from bisect import bisect_left
from typing import List
import pandas as pd
import scipy.stats as ss
from scipy.stats import ranksums
#correct if the population S.D. is expected to be equal for the two groups.
def cohen_d(x,y):
    nx = len(x)
    ny = len(y)
    dof = nx + ny - 2
    return (mean(x) - mean(y)) / sqrt(((nx-1)*std(x, ddof=1) ** 2 + (ny-1)*std(y, ddof=1) ** 2) / dof)
def VD_A(treatment: List[float], control: List[float]):
    """
    Computes Vargha and Delaney A index
    A. Vargha and H. D. Delaney.
    A critique and improvement of the CL common language
    effect size statistics of McGraw and Wong.
    Journal of Educational and Behavioral Statistics, 25(2):101-132, 2000
    The formula to compute A has been transformed to minimize accuracy errors
    See: http://mtorchiano.wordpress.com/2014/05/19/effect-size-of-r-precision/
    :param treatment: a numeric list
    :param control: another numeric list
    :returns the value estimate and the magnitude
    """
    m = len(treatment)
    n = len(control)

    if m != n:
        raise ValueError("Data d and f must have the same length")

    r = ss.rankdata(treatment + control)
    r1 = sum(r[0:m])

    # Compute the measure
    # A = (r1/m - (m+1)/2)/n # formula (14) in Vargha and Delaney, 2000
    A = (2 * r1 - m * (m + 1)) / (2 * n * m)  # equivalent formula to avoid accuracy errors

    levels = [0.147, 0.33, 0.474]  # effect sizes from Hess and Kromrey, 2004
    magnitude = ["negligible", "small", "medium", "large"]
    scaled_A = (A - 0.5) * 2

    magnitude = magnitude[bisect_left(levels, abs(scaled_A))]
    estimate = A

    return estimate, magnitude







def get_objectives(folders):
    objectives_list = []
    for sub_folder_path in sorted(folders):
        cur_info_data_path = os.path.join(sub_folder_path, 'cur_info.pickle')
        with open(cur_info_data_path, 'rb') as f_in:
            cur_info = pickle.load(f_in)
        objectives = cur_info['objectives']
        objectives_list.append(objectives)
    objectives_np = np.array(objectives_list)
    return objectives_np









def get_errors_count(folders, last_n_frame_id):
    error_count_list = []
    fusion_error_count_list = []
    fusion_error_cam_count_list = []
    fusion_error_radar_count_list = []

    min_d_old_list = []
    min_d_list = []
    mean_d_list = []
    print('\n'*3)
    print('category, sub_folder, total_count, error_count, fusion_error_both_count, fusion_error_cam_count, fusion_error_radar_count, fusion_error_count/total_count')
    for sub_folder_path in sorted(folders):
        simulation_data_path = os.path.join(sub_folder_path, 'simulation_data.pickle')
        simulation_data_formated_path = os.path.join(sub_folder_path, 'simulation_data_formated.txt')
        with open(simulation_data_path, 'rb') as f_in:
            simulation_data = pickle.load(f_in)

        keys_intersected_sorted = get_lead_interseced_keys(simulation_data)

        total_count, error_count, fusion_error_both_count, fusion_error_cam_count, fusion_error_radar_count = record_lead_and_error(simulation_data, simulation_data_formated_path, keys_intersected_sorted, last_n_frame_id=last_n_frame_id)

        min_d_old = np.min(list(simulation_data['d_avg_old_list_dict'].values()))
        min_d = np.min(list(simulation_data['d_avg_list_dict'].values()))
        mean_d = np.mean(list(simulation_data['d_avg_list_dict'].values()))

        fusion_error_count = fusion_error_both_count + fusion_error_cam_count + fusion_error_radar_count
        if total_count > 0:
            print(category, sub_folder_path[-3:], total_count, error_count, fusion_error_both_count, fusion_error_cam_count, fusion_error_radar_count, fusion_error_count/total_count)

            error_count_list.append(error_count/total_count)
            fusion_error_count_list.append(fusion_error_count/total_count)
            fusion_error_cam_count_list.append(fusion_error_cam_count/total_count)
            fusion_error_radar_count_list.append(fusion_error_radar_count/total_count)

            min_d_old_list.append(min_d_old)
            min_d_list.append(min_d)
            mean_d_list.append(mean_d)

    return error_count_list, fusion_error_count_list, fusion_error_cam_count_list, fusion_error_radar_count_list, min_d_old_list, min_d_list, mean_d_list


def draw_field(parent_folders_results, title):
    styles = {
    'fusion errors': '-',
    'non-fusion errors': '--',
    'normal': ':'
    }
    a4_dims = (16/3, 4)
    fig, ax = plt.subplots(figsize=a4_dims)

    for category, results in parent_folders_results.items():
        xlabel = title
        if title == 'fusion percent error':
            bin_num = 20
            range = (0, 1)
            xlabel = r'$\mathbf{F}_{fusion}$'
        elif title == 'min d':
            bin_num = 20
            range = (-5, 15)

        ax.set_xlim(range[0], range[1])

        sns.kdeplot(np.array(results[title]),
                 linewidth=3,
                 label=category,
                 linestyle=styles[category])


        # plt.hist(np.array(results[title]), density=True, label=category, bins=bin_num, range=range, alpha=0.5)

    plt.legend(prop={'size': 20}, framealpha=0.3)
    # plt.title('pdf of '+title,fontsize=25)
    plt.xlabel(xlabel,fontsize=25)
    plt.ylabel('density',fontsize=25)
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)
    plt.savefig(title+'.pdf', transparent=True, bbox_inches='tight', pad_inches=0)
    plt.clf()



if __name__ == '__main__':

    last_n_frame_id = 2.5 / 0.01

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-p",
        "--parent_folder",
        type=str,
        default="",
        help="parent_folder to run analysis of fusion error",
    )
    parser.add_argument(
        "-t",
        "--task",
        type=str,
        default="hist",
        help="'hist', 'stat'",
    )



    arguments = parser.parse_args()
    parent_folder = arguments.parent_folder
    task = arguments.task
    assert task in ['hist', 'stat']

    original_folder = os.path.join(parent_folder, 'original')
    original_bugs_folder = os.path.join(original_folder, 'bugs')
    original_non_bugs_folder = os.path.join(original_folder, 'non_bugs')

    rerun_folder = os.path.join(parent_folder, 'rerun_2.5_best_sensor')
    rerun_bugs_folder = os.path.join(rerun_folder, 'bugs')
    rerun_non_bugs_folder = os.path.join(rerun_folder, 'non_bugs')


    normal_folders = [os.path.join(original_non_bugs_folder, subfolder) for subfolder in os.listdir(original_non_bugs_folder)]
    fusion_bugs_folders = []
    non_fusion_bugs_folders = []


    for subfolder in os.listdir(rerun_bugs_folder):
        cur_folder = os.path.join(original_bugs_folder, subfolder)
        if os.path.isdir(cur_folder):
            non_fusion_bugs_folders.append(cur_folder)

    for subfolder in os.listdir(rerun_non_bugs_folder):
        cur_folder = os.path.join(original_bugs_folder, subfolder)
        if os.path.isdir(cur_folder):
            fusion_bugs_folders.append(cur_folder)



    # draw fusion/non fusion/normal state values(fusion percent error, min d) histograms
    if task == 'hist':
        parent_folders = {
            'normal': normal_folders,
            'non-fusion errors': non_fusion_bugs_folders,
            'fusion errors': fusion_bugs_folders
        }

        parent_folders_results = {

        }

        for category, folders in parent_folders.items():
            error_count_list, fusion_error_count_list, fusion_error_cam_count_list, fusion_error_radar_count_list, min_d_old_list, min_d_list, mean_d_list = get_errors_count(folders, last_n_frame_id)

            print('category', category)
            print('np.mean(error_count_list)', np.mean(error_count_list))
            print('np.mean(fusion_error_count_list)', np.mean(fusion_error_count_list))

            parent_folders_results[category] = {
                # 'error': error_count_list,
                'fusion percent error': fusion_error_count_list,
                # 'fusion percent error cam': fusion_error_cam_count_list,
                # 'fusion percent error radar': fusion_error_radar_count_list,
                'min d': min_d_list,
                # 'mean d': mean_d_list,
                # 'min d old': min_d_old_list
            }

        for title, _ in parent_folders_results['normal'].items():
            draw_field(parent_folders_results, title)

    # statistical test of objectives
    elif task == 'stat':
        objectives_normal = get_objectives(normal_folders)
        objectives_non_fusion = get_objectives(non_fusion_bugs_folders)
        objectives_fusion = get_objectives(fusion_bugs_folders)

        fusion_perc_error_non_fusion = objectives_non_fusion[:, 5]
        fusion_perc_error_fusion = objectives_fusion[:, 5]

        print ("cohen d = ", cohen_d(fusion_perc_error_fusion, fusion_perc_error_non_fusion))
        print('Wilcoxon rank-sum statistics p value', ranksums(fusion_perc_error_fusion, fusion_perc_error_non_fusion))
        print('Vargha-Delaney effect size', VD_A(fusion_perc_error_fusion, fusion_perc_error_non_fusion))
