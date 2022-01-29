

from tools.sim.op_script.utils_multiple import append_relevant_paths
if __name__ == "__main__":
    append_relevant_paths()


import socket
import sys
import os
from pathlib import Path
import argparse
import seaborn as sns
from matplotlib import pyplot as plt
import re
import pickle
import numpy as np
from tools.sim.op_script.op_specific import is_distinct_vectorized


















def extract_ind(folder):
    folder_ind = re.search('.*/([0-9]+)$', folder).group(1)
    return int(folder_ind)

def extract_subfolder_inds_for_parent_folder(folder):
    return sorted([int(subfolder) for subfolder in os.listdir(folder)])


def sort_subfolders(original_folder):
    folders = []
    for subfolder in os.listdir(original_folder):
        subfolder_path = os.path.join(original_folder, subfolder)
        if os.path.isdir(subfolder_path):
            folders.append(subfolder_path)
    folders = sorted(folders, key=extract_ind)
    return folders

def plot_collision_locations_over_gen(original_bugs_folder):
    bugs_folders = sort_subfolders(original_bugs_folder)
    print(bugs_folders)
    x_list = []
    y_list = []
    colors_list = []
    for bug_folder in bugs_folders:
        bug_folder_ind = extract_ind(bug_folder)

        collision_data_path = os.path.join(bug_folder, 'tmp_data.txt')
        if os.path.exists(collision_data_path):
            with open(collision_data_path, 'r') as f_in:
                tokens = f_in.read().split(',')
                x_list.append(float(tokens[1]))
                y_list.append(float(tokens[2]))
                colors_list.append(1+bug_folder_ind//100)
    plt.scatter(y_list, x_list, c=colors_list, s=10, alpha=0.7, cmap='Reds')
    plt.savefig('tmp.pdf')


def plot_trajectory_heatmap(folders):


    trajectory_list = []
    trajectory_unflatten_list = []

    for folder in sorted(folders):
        folder_ind = extract_ind(folder)
        cur_info_data_path = os.path.join(folder, 'cur_info.pickle')
        if os.path.exists(cur_info_data_path):
            with open(cur_info_data_path, 'rb') as f_in:
                print('cur_info_data_path', cur_info_data_path)
                cur_info_data = pickle.load(f_in)
            route_interval_num = len(cur_info_data['sim_specific_arguments'].kd_tree.data)

            trajectory_list.append(cur_info_data['trajectory_vector'].reshape(route_interval_num, -1))
            trajectory_unflatten_list.append(cur_info_data['trajectory_vector'])


    trajectory_np = np.array(trajectory_list)
    trajectory_np_unflatten = np.array(trajectory_unflatten_list)

    # from sklearn.preprocessing import StandardScaler, MinMaxScaler
    # scaler = StandardScaler()
    # normalizer = MinMaxScaler()


    # scaler.fit(trajectory_np_unflatten)
    # trajectory_np_unflatten = scaler.transform(trajectory_np_unflatten)

    # normalizer.fit(trajectory_np_unflatten)
    # trajectory_np_unflatten = normalizer.transform(trajectory_np_unflatten)

    # trajectory_np = trajectory_np_unflatten.reshape((trajectory_np_unflatten.shape[0], route_interval_num, -1))




    # plot heatmap
    trajectory_data_heatmap = np.mean(trajectory_np, axis=0)
    plt.tight_layout()
    ax = sns.heatmap(trajectory_data_heatmap, square=True)
    ax.invert_yaxis()
    ax.set_title('spatial-speed coverage heatmap')
    ax.set_xlabel('speed')
    ax.set_ylabel('location')
    plt.savefig('traj_heatmap.pdf')
    plt.close()

    # estimate mean avg diff
    diff_avg_list = []
    for i in range(trajectory_np_unflatten.shape[0]):
        diff = np.sum(np.abs(trajectory_np_unflatten - trajectory_np_unflatten[i]), axis=1)
        diff_avg = np.mean(diff)
        diff_avg_list.append(diff_avg)


    print('diff_avg_list', diff_avg_list)
    plt.hist(diff_avg_list, bins=50)
    plt.title('frequency of diff_avg')
    plt.xlabel('diff_avg')
    plt.ylabel('frequency')
    plt.savefig('diff_avg_dist.pdf')
    plt.close()

    diff_avg_all = np.mean(diff_avg_list)
    print('diff_avg_all', diff_avg_all)


    # estimate mean min diff
    from sklearn.neighbors import KDTree
    kd_tree = KDTree(trajectory_np_unflatten, leaf_size=1, metric='manhattan')
    diff_min_list = []
    for i in range(trajectory_np_unflatten.shape[0]):
        dists, inds = kd_tree.query(np.expand_dims(trajectory_np_unflatten[i], axis=0), k=2)
        diff_min_list.append(dists[0][1])
        print('folder ind, dists[0][1], inds[0][1]', extract_ind(folders[i]), dists[0][1], extract_ind(folders[inds[0][1]]))

    plt.hist(diff_min_list, bins=50)
    plt.title('frequency of diff_min')
    plt.xlabel('diff_min')
    plt.ylabel('frequency')
    plt.savefig('diff_min_dist.pdf')
    plt.close()

    diff_min_all = np.mean(diff_min_list)
    diff_min_all_std = np.std(diff_min_list)
    print('diff_min_all', diff_min_all)
    print('diff_min_all_std', diff_min_all_std)



    # count number of unique bugs
    n_cov = trajectory_np_unflatten.shape[1]
    mask_in_effect = ['int']*n_cov
    xl_in_effect = [0]*n_cov
    xu_in_effect = [1]*n_cov
    p = 0
    c = 0.5
    # hack: need to be customizable at interface as that in op_specific - get_unique_bugs
    print('trajectory_np_unflatten.shape[1]', trajectory_np_unflatten.shape[1])
    th = 1 / trajectory_np_unflatten.shape[1]

    remaining_inds = is_distinct_vectorized(trajectory_np_unflatten, [], mask_in_effect, xl_in_effect, xu_in_effect, p, c, th, verbose=True)

    print('remaining_inds', len(remaining_inds), remaining_inds)
    print('remaining_inds folders', np.array(folders)[remaining_inds])
    print(trajectory_np[remaining_inds])




if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-p",
        "--parent_folder",
        type=str,
        default="",
        help="parent_folder to run analysis of fusion error",
    )
    parser.add_argument(
        "-f",
        "--fusion_folder",
        type=str,
        default="",
        help="folder consists of the fusion error folders in rerun; it is used to filter in only fusion error paths in the parent folder",
    )

    arguments = parser.parse_args()
    parent_folder = arguments.parent_folder


    # plot_collision_locations_over_gen
    # original_bugs_folder = os.path.join(parent_folder, 'bugs')
    # plot_collision_locations_over_gen(original_bugs_folder)

    # plot trajectory heatmap

    folders = sort_subfolders(parent_folder)
    if arguments.fusion_folder:
        fusion_inds = extract_subfolder_inds_for_parent_folder(arguments.fusion_folder)
        fusion_folders = []
        for subfolder in folders:
            cur_ind = extract_ind(subfolder)
            if cur_ind in fusion_inds:
                fusion_folders.append(subfolder)
        folders = fusion_folders
    print('len(folders)', len(folders))
    plot_trajectory_heatmap(folders)
