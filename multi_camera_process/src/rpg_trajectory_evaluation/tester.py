import os
import argparse
from argparse import Namespace
import sys
sys.path.insert(1, './src/rpg_trajectory_evaluation/scripts/')
sys.path.insert(1, './src/rpg_trajectory_evaluation')

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc
import matplotlib.font_manager
import matplotlib
from colorama import init, Fore
import pandas as pd
from scipy.spatial.transform import Rotation as R
from PIL import Image
from matplotlib.backends.backend_pdf import PdfPages

import src.rpg_trajectory_evaluation
from trajectory import Trajectory
import plot_utils as pu
from fn_constants import kNsToEstFnMapping, kNsToMatchFnMapping, kFnExt
from multiple_traj_errors import MulTrajError

from evo.tools import plot as evo_plot
from evo.core import trajectory
from evo.tools.plot import PlotMode
from evo.core.metrics import PoseRelation, Unit
from evo.core import metrics
from evo.tools.settings import SETTINGS
import evo.main_ape as main_ape
import evo.common_ape_rpe as common
from evo.tools import log
log.configure_logging()

SETTINGS.plot_figsize = [6, 6]
SETTINGS.plot_split = True
SETTINGS.plot_usetex = False

import copy
import warnings
warnings.filterwarnings("ignore")
plt.ioff()

init(autoreset=True)
rc('font', **{'family': 'serif', 'serif': ['Cardo']})
rc('text', usetex=True)

def fig2data(fig):
    fig.canvas.draw()
    w,h = fig.canvas.get_width_height()
    buf = np.fromstring(fig.canvas.tostring_argb(), dtype=np.uint8)
    buf.shape = (w, h, 4)
    buf = np.roll (buf, 3, axis = 2)
    return buf

def fig2img(fig):
    buf = fig2data(fig)
    w, h, d = buf.shape
    return Image.frombytes("RGBA", (w ,h), buf.tostring())

class trajEval():
    def __init__(self,
                 trajectories,
                 trajectory_names = None,
                 align='first_frame',
                 relative_segment_lengths = [100, 200, 300, 400, 500, 600, 700, 800],
                 mean_over_all = True,
                 store=None,
                 use_cached=False):
        
        argspace = {
            'mul_trials': None, # whether multiple trajectoreis need to be parsed
            'mul_plot_idx': [0],
            'est_type': 'traj_est', # mode of estimate
            'recalculate_errors': not use_cached, # whether to remove the old cached files
            'png': False, # save plots as png rather than pdf
            'plot_scale_traj': True, # whether to plot scale colored trajectory (slow)
            'n_trials': 1,
            'preset_boxplot_distances': relative_segment_lengths,
            'alignment_type': align,
            'mean_over_all': mean_over_all,
            'store': store,
            'trajectory_names': trajectory_names,
            'trajectory_paths': [],
            'raw_trajectory_paths': []
        }
        args = Namespace(**argspace)
        
        assert args.est_type in kNsToEstFnMapping
        assert args.est_type in kNsToMatchFnMapping
        if args.store is not None:
            assert os.path.exists(args.store)
            args.store = os.path.join(args.store, 'results')
            if not os.path.exists(args.store):
                os.makedirs(args.store)
        if args.png:
            self.FORMAT = '.png'
        else:
            self.FORMAT = '.pdf'
            
        if args.trajectory_names is None:
            args.trajectory_names = []
            for idx in range(len(trajectories)):
                args.trajectory_names.append('Traj-{}'.format(idx))
        
        if args.store is not None:
            for name in args.trajectory_names:
                traj_path_temp = args.store+'/'+str(name)
                args.trajectory_paths.append(traj_path_temp)
                if not os.path.exists(traj_path_temp+'/relative'):
                    os.makedirs(traj_path_temp+'/relative')
                if not os.path.exists(traj_path_temp+'/absolute'):
                    os.makedirs(traj_path_temp+'/absolute')
                if not os.path.exists(traj_path_temp+'/path'):
                    os.makedirs(traj_path_temp+'/path')
        else:
            for name in args.trajectory_names:
                traj_path_temp = './'+str(name)
                args.trajectory_paths.append(traj_path_temp)
                if not os.path.exists(traj_path_temp):
                    os.makedirs(traj_path_temp)
                
        self.args = args
        self.trajectories = trajectories
        self.processed = []
        self.process()
        
        
    def process(self):
        self.all_results = {}
        for traj, name, path, in zip(self.trajectories,
                                     self.args.trajectory_names,
                                     self.args.trajectory_paths):
            targets, predictions = traj
            
            self.traj_ref = trajectory.PoseTrajectory3D(poses_se3=targets, timestamps=np.arange(targets.shape[0]))
            self.traj_est = trajectory.PoseTrajectory3D(poses_se3=predictions, timestamps=np.arange(predictions.shape[0]))
            self.traj_est_scaled = trajectory.align_trajectory(self.traj_est, self.traj_ref, correct_only_scale=True)
            self.traj_est_aligned = trajectory.align_trajectory(self.traj_est, self.traj_ref)
            self.traj_est_aligned_scaled = trajectory.align_trajectory(self.traj_est, self.traj_ref, correct_scale=True)
            
            R_prd = predictions[:, :3, :3]
            t_prd = predictions[:, :3,  3]
            R_trg = targets[:, :3, :3]
            t_trg = targets[:, :3,  3]
            q_prd = R.from_matrix(R_prd).as_quat()
            q_trg = R.from_matrix(R_trg).as_quat()
            prd = np.hstack((np.arange(R_prd.shape[0]).reshape(-1, 1), t_prd, q_prd))
            trg = np.hstack((np.arange(R_trg.shape[0]).reshape(-1, 1), t_trg, q_trg))
            
            raw_traj_path = (path+'/stamped_groundtruth.txt',
                             path+'/stamped_traj_estimate.txt')
            self.args.raw_trajectory_paths.append(raw_traj_path)
            np.savetxt(path+'/stamped_groundtruth.txt', trg, delimiter=' ')
            np.savetxt(path+'/stamped_traj_estimate.txt', prd, delimiter=' ')
            
            traj_list, mt_error = self.analyze_multiple_trials(results_dir=path,
                                                               est_type = self.args.est_type,
                                                               n_trials = 1,
                                                               align_type = self.args.alignment_type,
                                                               recalculate_errors = self.args.recalculate_errors,
                                                               preset_boxplot_distances = self.args.preset_boxplot_distances)
            suffix = ''
            plot_types = ['rel_trans', 'rel_trans_perc', 'rel_yaw']
            labels = ['Estimate']
            colors = ['b']
            rel_errors, distances = mt_error.get_relative_errors_and_distances(error_types=plot_types)
            self.processed.append(traj_list[0])
            
            if traj_list:
                plot_traj = traj_list[self.args.mul_plot_idx[0]]
                results = traj_list[0]
            else:
                print("No successful runs, not plotting.")
            
            rc("text", usetex=False)
            rel_stats = self.rel_stat_generator(results)
            abs_stats = self.abs_stat_generator(results)
            relative_histograms = self.rel_hist_generator(results, path)
            relative_tplots = self.rel_tplot_generator(results, path)
            absolute_tplots = self.abs_tplot_generator(results, path)
            estimate_mappings = self.estimate_mappings_generator(plot_traj, path)
            position_drift = self.position_drift_generator(plot_traj, path)
            yaw_drift = self.yaw_drift_generator(plot_traj, path)
            scale_drift = self.scale_drift_generator(plot_traj, path)
            rel_translation_boxplot = self.rel_translation_boxplot_generator(rel_errors, distances, path)
            rel_translation_precentage_boxplot = self.rel_translation_precentage_boxplot_generator(rel_errors, distances, path)
            rel_yaw_percentage = self.rel_yaw_percentage_generator(rel_errors, distances, path)
            abs_path_plot = self.abs_path_plot_generator(path)
            rel_path_plot = self.rel_path_plot_generator(path)
            
            self.all_results.update({name: {'relative_stats': rel_stats,
                                            'absolute_stats': abs_stats,
                                            'relative_histograms': relative_histograms,
                                            'relative_tplots': relative_tplots,
                                            'absolute_tplots': absolute_tplots,
                                            'estimate_mappings': estimate_mappings,
                                            'position_drift': position_drift,
                                            'yaw_drift': yaw_drift,
                                            'rel_translation_boxplot': rel_translation_boxplot,
                                            'rel_translation_precentage_boxplot': rel_translation_precentage_boxplot,
                                            'rel_yaw_percentage': rel_yaw_percentage,
                                            'abs_path_plot': abs_path_plot,
                                            'rel_path_plot': rel_path_plot,}})
        
        relative_mean = {}
        for length in self.args.preset_boxplot_distances:
            relative_mean.update({length: []})
            
        for name, value in self.all_results.items():
            for k, v in self.all_results[name]['relative_stats'].items():
                columns = self.all_results[name]['relative_stats'][k].columns
                indexes = self.all_results[name]['relative_stats'][k].index
                relative_mean[k].append(self.all_results[name]['relative_stats'][k].values)

        for length in self.args.preset_boxplot_distances:
            relative_mean[length] = pd.DataFrame(np.stack(relative_mean[length]).mean(0), index=indexes, columns=columns)
            
        self.all_results.update({'mean': relative_mean})
            
    def rel_stat_generator(self, results):
        rel_stats = {}
        for k, v in results.rel_errors.items():
            df = pd.DataFrame.from_dict({ki: vi for ki, vi in results.rel_errors[k].items() if 'stats' in ki})
            rel_stats.update({k: df})
        return rel_stats
    
    def store_relative_tables(self):
        for name, name_val in self.all_results.items():
            if name is not 'mean':
                for tlen, value in self.all_results[name]['relative_stats'].items():
                    df = self.all_results[name]['relative_stats'][tlen]
                    fig, ax =plt.subplots(figsize=(12,4))
                    ax.axis('tight'); ax.axis('off')
                    the_table = ax.table(cellText=df.values,colLabels=df.columns,loc='center')
                    pp = PdfPages(self.args.store+'/'+name+'/relative/'+name+"-relative-stats-"+str(tlen)+"m.pdf")
                    pp.savefig(fig, bbox_inches='tight')
                    pp.close()
                    plt.close()
        for name, name_val in self.all_results.items():
            for tlen, value in self.all_results['mean'].items():
                df = self.all_results['mean'][tlen]
                fig, ax =plt.subplots(figsize=(12,4))
                ax.axis('tight'); ax.axis('off')
                the_table = ax.table(cellText=df.values,colLabels=df.columns,loc='center')
                pp = PdfPages(self.args.store+'/'+"mean-relative-stats-"+str(tlen)+"m.pdf")
                pp.savefig(fig, bbox_inches='tight')
                pp.close()
                plt.close()
    
    def abs_stat_generator(self, results):
        abs_stats = {}
        for k, v in results.abs_errors.items():
            if 'stats' in k:
                abs_stats.update({k: v})
        abs_stats = pd.DataFrame.from_dict(abs_stats)
        return abs_stats
    
    def store_absolute_table(self):
        for name, name_val in self.all_results.items():
            if name is not 'mean':
                df = self.all_results[name]['absolute_stats']
                fig, ax =plt.subplots(figsize=(12,4))
                ax.axis('tight'); ax.axis('off')
                the_table = ax.table(cellText=df.values,colLabels=df.columns,loc='center')
                pp = PdfPages(self.args.store+'/'+str(name)+'/absolute/'+name+"-absolute-stats.pdf")
                pp.savefig(fig, bbox_inches='tight')
                pp.close()
                plt.close()
    
    def rel_hist_generator(self, results, path):
        output = {}
        for tlen, teln_value in results.rel_errors.items():
            fig = plt.figure(figsize=(10, 20))
            fig.patch.set_facecolor('white')
            axs = []
            idx = 1
            for k, v in results.rel_errors[tlen].items():
                if 'stats' not in k:
                    ax = fig.add_subplot(6, 1, idx)
                    title = k + ' for '+str(tlen)+' meters'
                    ax.title.set_text(title)
                    ax.hist(results.rel_errors[tlen][k])
                    idx = idx + 1
            plt.tight_layout()
            image = fig2img(fig)
            output.update({tlen: image})
            if self.args.store is not None:
                plt.savefig(path+'/relative/rel-hist-{}m'.format(tlen)+self.FORMAT)
            plt.close()
        return output
    
    def rel_tplot_generator(self, results, path):
        output = {}
        for tlen, teln_value in results.rel_errors.items():
            fig = plt.figure(figsize=(10, 20))
            fig.patch.set_facecolor('white')
            axs = []
            idx = 1
            for k, v in results.rel_errors[tlen].items():
                if 'stats' not in k:
                    ax = fig.add_subplot(6, 1, idx)
                    title = k + ' for '+str(tlen)+' meters'
                    ax.title.set_text(title)
                    ax.plot(results.rel_errors[tlen][k])
                    idx = idx + 1
            plt.tight_layout()
            image = fig2img(fig)
            output.update({tlen: image})
            if self.args.store is not None:
                plt.savefig(path+'/relative/rel-tplot-{}m'.format(tlen)+self.FORMAT)
            plt.close()
        return output
    
    def abs_tplot_generator(self, results, path):
        fig = plt.figure(figsize=(10, 20))
        fig.patch.set_facecolor('white')
        axs = []
        idx = 1
        for k, v in results.abs_errors.items():
            if 'stats' not in k:
                ax = fig.add_subplot(6, 1, idx)
                title = k
                ax.title.set_text(title)
                ax.plot(results.abs_errors[k])
                idx = idx + 1
        plt.tight_layout()
        output = fig2img(fig)
        if self.args.store is not None:
            plt.savefig(path+'/absolute/abs-tplot'+self.FORMAT)
        plt.close()
        return output
    
    def estimate_mappings_generator(self, plot_traj, path):
        fig = plt.figure(figsize=(15, 7))
        fig.patch.set_facecolor('white')
        ax = fig.add_subplot(111, xlabel='x [m]', ylabel='y [m]')
        pu.plot_trajectory_top(ax, plot_traj.p_es_aligned, 'b', 'Estimate')
        pu.plot_trajectory_top(ax, plot_traj.p_gt, 'm', 'Groundtruth')
        pu.plot_aligned_top(ax, plot_traj.p_es_aligned, plot_traj.p_gt,
                            plot_traj.align_num_frames)

        plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
        plt.tight_layout()
        output = fig2img(fig)
        if self.args.store is not None:
            plt.savefig(path+'/path/trajectory_top' + '_' + plot_traj.align_str + self.FORMAT)
        plt.close()
        return output
    
    def position_drift_generator(self, plot_traj, path):
        fig = plt.figure(figsize=(10, 4))
        fig.patch.set_facecolor('white')
        ax = fig.add_subplot(
            111, xlabel='Distance [m]', ylabel='Position Drift [mm]',
            xlim=[0, plot_traj.accum_distances[-1]])
        pu.plot_error_n_dim(ax, plot_traj.accum_distances,
                            plot_traj.abs_errors['abs_e_trans_vec']*1000,
                            path)
        ax.legend()
        plt.tight_layout()
        output = fig2img(fig)
        if self.args.store is not None:
            plt.savefig(path+'/path/translation_error' + '_' + plot_traj.align_str + self.FORMAT, bbox_inches="tight")
        plt.close()
        return output
    
    def yaw_drift_generator(self, plot_traj, path):
        fig = plt.figure(figsize=(10, 5))
        fig.patch.set_facecolor('white')
        ax = fig.add_subplot(111, xlabel='Distance [m]', ylabel='Orient. err. [deg]', xlim=[0, plot_traj.accum_distances[-1]])
        pu.plot_error_n_dim(ax, plot_traj.accum_distances, plot_traj.abs_errors['abs_e_ypr']*180.0/np.pi, path, labels=['yaw', 'pitch', 'roll'])
        ax.legend()
        plt.tight_layout()
        output = fig2img(fig)
        if self.args.store is not None:                    
            plt.savefig(path+'/path/rotation_error'+'_'+plot_traj.align_str + self.FORMAT, bbox_inches='tight')
        plt.close()
        return output
                            
    def scale_drift_generator(self, plot_traj, path):
        fig = plt.figure(figsize=(10, 5))
        fig.patch.set_facecolor('white')
        ax = fig.add_subplot(111, xlabel='Distance [m]', ylabel='Scale Drift [\%]', xlim=[0, plot_traj.accum_distances[-1]])
        pu.plot_error_n_dim(ax, plot_traj.accum_distances, np.reshape(plot_traj.abs_errors['abs_e_scale_perc'], (-1, 1)), path, colors=['b'], labels=['scale'])
        ax.legend()
        plt.tight_layout()
        output = fig2img(fig)
        if self.args.store is not None:
            plt.savefig(path+'/path/scale_error'+'_'+plot_traj.align_str+self.FORMAT, bbox_inches='tight')
        plt.close()
        return output
                            
    def rel_translation_boxplot_generator(self, rel_errors, distances, path):
        suffix = ''
        plot_types = ['rel_trans', 'rel_trans_perc', 'rel_yaw']
        labels = ['Estimate']
        colors = ['b']
        fig = plt.figure(figsize=(10, 5))
        fig.patch.set_facecolor('white')
        ax = fig.add_subplot(111, xlabel='Distance traveled [m]', ylabel='Translation error [m]')
        pu.boxplot_compare(ax, distances, rel_errors['rel_trans'], labels, colors)
        plt.tight_layout()
        output = fig2img(fig)
        if self.args.store is not None:
            plt.savefig(path+'/relative/rel_translation_error' + suffix + self.FORMAT, bbox_inches="tight")
        plt.close()
        return output
                            
    def rel_translation_precentage_boxplot_generator(self, rel_errors, distances, path):
        suffix = ''
        plot_types = ['rel_trans', 'rel_trans_perc', 'rel_yaw']
        labels = ['Estimate']
        colors = ['b']
        fig = plt.figure(figsize=(10, 5))
        fig.patch.set_facecolor('white')
        ax = fig.add_subplot(111, xlabel='Distance traveled [m]', ylabel='Translation error [\%]')
        pu.boxplot_compare(ax, distances, rel_errors['rel_trans_perc'], labels, colors)
        plt.tight_layout()
        output = fig2img(fig)
        if self.args.store is not None:
            plt.savefig(path+'/relative/rel_translation_error_perc'+suffix+self.FORMAT, bbox_inches="tight")
        plt.close()
        return output
                            
    def rel_yaw_percentage_generator(self, rel_errors, distances, path):
        suffix = ''
        plot_types = ['rel_trans', 'rel_trans_perc', 'rel_yaw']
        labels = ['Estimate']
        colors = ['b']
        fig = plt.figure(figsize=(10, 5))
        fig.patch.set_facecolor('white')
        ax = fig.add_subplot(111, xlabel='Distance traveled [m]', ylabel='Yaw error [deg]')
        pu.boxplot_compare(ax, distances, rel_errors['rel_yaw'], labels, colors)
        plt.tight_layout()
        output = fig2img(fig)
        if self.args.store is not None:                    
            plt.savefig(path+'/relative/rel_yaw_error' + suffix + self.FORMAT, bbox_inches="tight")
        plt.close()
        return output
                            
    def abs_path_plot_generator(self, path):
        pose_relation = metrics.PoseRelation.translation_part
        data = (self.traj_ref, self.traj_est)
        ape_metric = metrics.APE(pose_relation)
        ape_metric.process_data(data)
        ape_stats = ape_metric.get_all_statistics()
        plot_mode = evo_plot.PlotMode.xz
        fig = plt.figure()
        fig.patch.set_facecolor('white')
        ax = evo_plot.prepare_axis(fig, plot_mode)
        evo_plot.traj(ax, plot_mode, self.traj_ref, '--', "gray", label="reference")
        evo_plot.traj(ax, plot_mode, self.traj_est, color="blue", label="Estimated")
        evo_plot.traj(ax, plot_mode, self.traj_est_aligned, color="green", label="Estimated (Aligned)")
        evo_plot.traj(ax, plot_mode, self.traj_est_scaled, color="red", label="Estimated (Scaled)")
        evo_plot.traj(ax, plot_mode, self.traj_est_aligned_scaled, color="orange", label="Estimated (Aligned-Scaled)")
        plt.title('Absolute Trajectory')
        ax.legend()
        output = fig2img(fig)
        if self.args.store is not None:
            plt.savefig(path+'/path/absolute_path_visualization'+self.FORMAT)
        plt.close()
        return output
                            
    def rel_path_plot_generator(self, path):
        delta = 100
        all_pairs = True
        delta_unit = metrics.Unit.meters
        pose_relation = metrics.PoseRelation.translation_part
        rpe_metric = metrics.RPE(pose_relation, delta, delta_unit, all_pairs)
        data = (self.traj_ref, self.traj_est)
        rpe_metric.process_data(data)
        rpe_stats = rpe_metric.get_all_statistics()

        traj_ref_plot = copy.deepcopy(self.traj_ref)
        traj_est_plot = copy.deepcopy(self.traj_est)
        traj_ref_plot.reduce_to_ids(rpe_metric.delta_ids)
        traj_est_plot.reduce_to_ids(rpe_metric.delta_ids)
        plot_mode = evo_plot.PlotMode.xz
        fig = plt.figure()
        fig.patch.set_facecolor('white')
        ax = evo_plot.prepare_axis(fig, plot_mode)
        evo_plot.traj(ax, plot_mode, traj_ref_plot, '--', "gray", "reference")
        evo_plot.traj_colormap(ax, traj_est_plot, rpe_metric.error, plot_mode, min_map=rpe_stats["min"], max_map=rpe_stats["max"], title='')
        ax.legend()
        output = fig2img(fig)
        if self.args.store is not None:
            plt.savefig(path+'/path/relative_path_visualization'+self.FORMAT)
        plt.close()
        return output
                            
    def analyze_multiple_trials(self, results_dir, est_type, n_trials,
                                recalculate_errors=True,
                                preset_boxplot_distances=[100, 200, 300, 400, 500, 600, 700, 800],
                                preset_boxplot_percentages=[],
                                compute_odometry_error=True,
                                align_type='first_frame'):
        traj_list = []
        mt_error = MulTrajError()
        for trial_i in range(n_trials):
            if n_trials == 1:
                suffix = ''
            else:
                suffix = str(trial_i)
            print("Trial {0}".format(trial_i))

            match_base_fn = kNsToMatchFnMapping[est_type]+suffix+'.'+kFnExt

            if recalculate_errors:
                Trajectory.remove_cached_error(results_dir,
                                               est_type, suffix)
                Trajectory.remove_files_in_save_dir(results_dir, est_type,
                                                    match_base_fn)
            traj = Trajectory(
                results_dir, est_type=est_type, suffix=suffix,
                nm_est=kNsToEstFnMapping[est_type] + suffix + '.'+kFnExt,
                nm_matches=match_base_fn,
                preset_boxplot_distances=preset_boxplot_distances,
                preset_boxplot_percentages=preset_boxplot_percentages,
                align_type=align_type)
            if traj.data_loaded:
                traj.compute_absolute_error()
                if compute_odometry_error:
                    traj.compute_relative_errors()
            if traj.success:
                traj.cache_current_error()
                traj.write_errors_to_yaml()

            if traj.success and not preset_boxplot_distances:
                print("Save the boxplot distances for next trials.")
                preset_boxplot_distances = traj.preset_boxplot_distances

            if traj.success:
                mt_error.addTrajectoryError(traj, trial_i)
                traj_list.append(traj)
            else:
                print("Trials {0} failed, will not count.".format(trial_i))
        mt_error.summary()
        mt_error.updateStatistics()
        return traj_list, mt_error