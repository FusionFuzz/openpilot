import sys
import argparse
import os
import numpy as np
import pickle
import seaborn as sns
sns.set_theme()
sns.set_style("white")

from matplotlib import pyplot as plt
from tools.sim.op_script.utils_multiple import append_relevant_paths
if __name__ == "__main__":
    append_relevant_paths()




from tools.sim.op_script.op_specific import is_distinct_vectorized
import re

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


def extract_trajectory_vector(folder):
    cur_info_data_path = os.path.join(folder, 'cur_info.pickle')
    with open(cur_info_data_path, 'rb') as f_in:
        cur_info_data = pickle.load(f_in)
    trajectory_vector = cur_info_data['trajectory_vector']
    return trajectory_vector


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-p",
        "--parent_folder",
        type=str,
        default="",
        help="parent folder",
    )

    arguments = parser.parse_args()
    parent_folder = arguments.parent_folder

    method_list = []

    for method_folder_name in os.listdir(parent_folder):
        method_folder = os.path.join(parent_folder, method_folder_name)
        method_list.append(method_folder_name)

    # 'show_traj_map', 'draw_num', 'individual_traj_dist'
    mode = 'draw_num'

    if mode == 'show_traj_map':
        ind = 445
        subpath = 'fusion_default/original/bugs/'+str(ind)+'/cur_info.pickle'
        cur_info_data_path = os.path.join(parent_folder, subpath)
        with open(cur_info_data_path, 'rb') as f_in:
            cur_info_data = pickle.load(f_in)
        route_interval_num = len(cur_info_data['sim_specific_arguments'].kd_tree.data)
        trajectory_map = cur_info_data['trajectory_vector'].reshape(route_interval_num, -1)
        print(trajectory_map)

        # plot heatmap
        a4_dims = (10, 10)
        fig, ax = plt.subplots(figsize=a4_dims)
        ax.set_aspect(1)
        trajectory_data_heatmap = trajectory_map
        plt.tight_layout()
        sns.heatmap(trajectory_data_heatmap, square=False, linewidths=0., ax=ax)
        ax.get_xaxis().set_visible(False)
        ax.get_yaxis().set_visible(False)
        fig.axes[1].set_visible(False)
        ax.invert_yaxis()
        # ax.set_title('spatial-speed coverage')
        # ax.set_xlabel('speed')
        # ax.set_ylabel('location')
        plt.savefig('traj_heatmap_'+str(ind)+'.pdf', transparent=True, bbox_inches='tight', pad_inches=0)
        plt.close()

    elif mode == 'individual_traj_dist':
        for method_name in method_list:

            original_folder = os.path.join(method_folder, 'original')
            rerun_folder = os.path.join(method_folder, 'rerun_2.5_best_sensor')

            print(method_name, original_folder, rerun_folder)
            original_bugs_folder = os.path.join(original_folder, 'bugs')
            rerun_non_bugs_folder = os.path.join(rerun_folder, 'non_bugs')

            fusion_errors_inds = np.array(extract_subfolder_inds_for_parent_folder(rerun_non_bugs_folder))

            trajectory_list = []
            for cur_fusion_error_ind in fusion_errors_inds:
                folder = os.path.join(original_bugs_folder, str(cur_fusion_error_ind))
                trajectory_list.append(extract_trajectory_vector(folder))
            trajectory_np = np.array(trajectory_list)

            for i in range(trajectory_np.shape[0]):
                dist_list = []
                for j in range(trajectory_np.shape[0]):
                    if j != i:
                        d = np.linalg.norm(np.abs(trajectory_np[i]-trajectory_np[j]), ord=0)
                        dist_list.append((d, fusion_errors_inds[j]))
                print(fusion_errors_inds[i], ': ', sorted(dist_list))


    elif mode == 'draw_num':
        colors_list = ['red', 'blue', 'purple']
        name_mapping = {
          'random': 'RANDOM',
          'fusion_default': 'GA-FUSION',
          'no_fusion': 'GA'
        }
        parent_folder_postfix = re.match('.*/(.*)$', parent_folder).group(1)
        for i, method_name in enumerate(method_list):

            errors_count_list_all = []
            fusion_errors_count_list_all = []
            distinct_fusion_errors_count_list_all = []

            color = colors_list[i]


            for j in range(1, 4):
                if j > 1:
                    cur_method_folder = os.path.join(parent_folder + '_' + str(j), method_name)
                else:
                    cur_method_folder = os.path.join(parent_folder, method_name)

                original_folder = os.path.join(cur_method_folder, 'original')
                rerun_folder = os.path.join(cur_method_folder, 'rerun_2.5_best_sensor')

                original_bugs_folder = os.path.join(original_folder, 'bugs')
                rerun_non_bugs_folder = os.path.join(rerun_folder, 'non_bugs')


                errors_inds = np.array(extract_subfolder_inds_for_parent_folder(original_bugs_folder))
                fusion_errors_inds = np.array(extract_subfolder_inds_for_parent_folder(rerun_non_bugs_folder))

                th_list = list(range(0, 550, 50))
                errors_count_list = []
                fusion_errors_count_list = []
                distinct_fusion_errors_count_list = []
                for th in th_list:
                    error_count = fusion_error_count = distinct_fusion_error_count = 0

                    cur_errors_inds = errors_inds[errors_inds < th]
                    cur_fusion_errors_inds = fusion_errors_inds[fusion_errors_inds < th]

                    trajectory_list = []
                    for cur_fusion_error_ind in cur_fusion_errors_inds:
                        folder = os.path.join(original_bugs_folder, str(cur_fusion_error_ind))
                        trajectory_list.append(extract_trajectory_vector(folder))
                    trajectory_np = np.array(trajectory_list)

                    if len(trajectory_np) > 0:
                        n_cov = trajectory_np.shape[1]
                        mask_in_effect = ['int']*n_cov
                        xl_in_effect = [0]*n_cov
                        xu_in_effect = [1]*n_cov
                        p = 0
                        c = 0.5
                        th = 1 / n_cov
                        remaining_inds = is_distinct_vectorized(trajectory_np, [], mask_in_effect, xl_in_effect, xu_in_effect, p, c, th, verbose=True)


                        error_count = len(cur_errors_inds)
                        fusion_error_count = len(cur_fusion_errors_inds)
                        distinct_fusion_error_count = len(remaining_inds)

                    errors_count_list.append(error_count)
                    fusion_errors_count_list.append(fusion_error_count)
                    distinct_fusion_errors_count_list.append(distinct_fusion_error_count)

                errors_count_list_all.append(errors_count_list)
                fusion_errors_count_list_all.append(fusion_errors_count_list)
                distinct_fusion_errors_count_list_all.append(distinct_fusion_errors_count_list)

            fusion_errors_count_np = np.array(fusion_errors_count_list_all)
            distinct_fusion_errors_count_np = np.array(distinct_fusion_errors_count_list_all)

            fusion_errors_count_np_mean = np.mean(fusion_errors_count_np, axis=0)
            fusion_errors_count_np_std = np.std(fusion_errors_count_np, axis=0)
            distinct_fusion_errors_count_np_mean = np.mean(distinct_fusion_errors_count_np, axis=0)
            distinct_fusion_errors_count_np_std = np.std(distinct_fusion_errors_count_np, axis=0)


            print('distinct_fusion_errors_count_list', distinct_fusion_errors_count_list)

            # plt.plot(th_list, fusion_errors_count_np_mean,  label=name_mapping[method_name], color=color, linestyle='--', alpha=0.5)
            # plt.plot(th_list, distinct_fusion_errors_count_np_mean,  label=name_mapping[method_name]+'*', color=color, linestyle='-', alpha=0.5)

            plt.errorbar(th_list, fusion_errors_count_np_mean, yerr=fusion_errors_count_np_std, label=name_mapping[method_name]+' fusion errors', color=color, linestyle='--', alpha=0.5, capsize=5)
            # plt.errorbar(th_list, distinct_fusion_errors_count_np_mean, yerr=distinct_fusion_errors_count_np_std, label=name_mapping[method_name]+' distinct fusion errors', color=color, linestyle='-', alpha=0.5, capsize=5)
            plt.xlabel('# Simulations', fontsize=20)
            plt.ylabel('# Errors', fontsize=20)
            plt.xticks(fontsize=18)
            plt.yticks(fontsize=18)

        tokens = parent_folder_postfix.split('_')

        map_mapping = {
        'town06': 'S1',
        'town04': 'S2',
        }
        method_mapping = {
        'mathwork': 'mathworks',
        'op': 'default'
        }
        map_name = map_mapping[tokens[0]]
        method_name = method_mapping[tokens[3]]
        title_name = map_name+' '+method_name
        plt.title(title_name, fontsize=22)
        plt.legend(prop={'size': 15}, framealpha=0.3)
        sns.despine()
        plt.savefig(parent_folder_postfix+'.pdf', transparent=True, bbox_inches='tight', pad_inches=0)
