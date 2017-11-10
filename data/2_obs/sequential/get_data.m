%--------------------------------------------------------------------------
%                           1 Obstacle data
%--------------------------------------------------------------------------


% Set filenames
fstr_compute_switch = 'compute_switch_ts.txt';
fstr_full_trajec    = 'full_trajectory.txt';
fstr_min_dist       = 'min_dist_obs.txt';
fstr_motion_error   = 'motion_error_amount.txt';
fstr_num_ccs        = 'num_ccs.txt';
fstr_num_pcs        = 'num_pcs.txt';
fstr_num_scs        = 'num_scs.txt';
fstr_cc_freq        = 'cc_freqs.txt';
fstr_pc_freq        = 'pc_freqs.txt';
fstr_sc_freq        = 'sc_freqs.txt';
fstr_pop_size       = 'pop_size.txt';
fstr_runtime        = 'runtime.txt';
fstr_switch_t_size  = 'switch_t_size.txt';
fstr_time_in_ic     = 'time_in_ic.txt';
fstr_trajec_size    = 'trajec_size.txt';



%--------------------------------------------------------------------------
% General data
%--------------------------------------------------------------------------
compute_switch_ts   = get_values(fstr_compute_switch);
full_trajec         = get_values(fstr_full_trajec);
min_dist_obs        = get_values(fstr_min_dist);
motion_error_amount = get_values(fstr_motion_error);
num_ccs             = get_values(fstr_num_ccs);
num_pcs             = get_values(fstr_num_pcs);
num_scs             = get_values(fstr_num_scs);
pc_freqs            = get_values(fstr_pc_freq);
cc_freqs            = get_values(fstr_cc_freq);
sc_freqs            = get_values(fstr_sc_freq);
pop_size            = get_values(fstr_pop_size);
runtime             = get_values(fstr_runtime);
switch_t_size       = get_values(fstr_switch_t_size);
time_in_ic          = get_values(fstr_time_in_ic);
trajec_size         = get_values(fstr_trajec_size);

compute_switch_ts_dist = fitdist(compute_switch_ts, 'Normal');
%full_trajec_dist = fitdist(full_trajec, 'Normal');
min_dist_obs_dist = fitdist(min_dist_obs, 'Normal');
motion_error_amount_dist = fitdist(motion_error_amount, 'Normal');
num_ccs_dist = fitdist(num_ccs, 'Normal');
num_pcs_dist = fitdist(num_pcs, 'Normal');
num_scs_dist = fitdist(num_scs, 'Normal');
pc_freqs_dist = fitdist(pc_freqs, 'Normal');
cc_freqs_dist = fitdist(cc_freqs, 'Normal');
sc_freqs_dist = fitdist(sc_freqs, 'Normal');
pop_size_dist = fitdist(pop_size, 'Normal');
runtime_dist = fitdist(runtime, 'Normal');
switch_t_size_dist = fitdist(switch_t_size, 'Normal');
time_in_ic_dist = fitdist(time_in_ic, 'Normal');
trajec_size_dist = fitdist(trajec_size, 'Normal');

%--------------------------------------------------------------------------
% Duration data
%--------------------------------------------------------------------------
disp('Duration data');
fstr_cc_durs        = 'cc_durs.txt';
fstr_pc_durs        = 'pc_durs.txt';
fstr_sc_durs        = 'sc_durs.txt';
fstr_durs_path_mods = 'durations_path_mods.txt';
fstr_durs_sensing   = 'durations_sensing.txt';
fstr_durs_trj_eval  = 'durations_trj_eval.txt';
fstr_durs_trj_gen   = 'durations_trj_gen.txt';
fstr_eval_durs      = 'eval_durs.txt';
fstr_trajec_durs    = 'trajec_durs.txt';
fstr_mod_durs       = 'mod_durs.txt';
fstr_mutate_durs    = 'mutate_durs.txt';
fstr_error_correct_durs_eval    = 'error_correct_durs_eval.txt';
fstr_error_correct_durs_no_eval = 'error_correct_durs_no_eval.txt';

cc_durs         = get_values(fstr_cc_durs);
pc_durs         = get_values(fstr_pc_durs);
sc_durs         = get_values(fstr_sc_durs);
durs_path_mods  = get_values(fstr_durs_path_mods);
durs_sensing    = get_values(fstr_durs_sensing);
durs_trj_eval   = get_values(fstr_durs_trj_eval);
durs_trj_gen    = get_values(fstr_durs_trj_gen);
eval_durs_mp    = get_values(fstr_eval_durs);
gen_durs_mp     = get_values(fstr_trajec_durs);
mod_durs_mp     = get_values(fstr_mod_durs);
mutate_durs_mp  = get_values(fstr_mutate_durs);
error_corr_durs_eval    = get_values(fstr_error_correct_durs_eval);
error_corr_durs_no_eval = get_values(fstr_error_correct_durs_no_eval);

cc_durs_dist        = fitdist(cc_durs, 'Normal');
pc_durs_dist        = fitdist(pc_durs, 'Normal');
sc_durs_dist        = fitdist(sc_durs, 'Normal');
durs_path_mods_dist = fitdist(durs_path_mods, 'Normal');
durs_sensing_dist   = fitdist(durs_sensing, 'Normal');
durs_trj_eval_dist  = fitdist(durs_trj_eval, 'Normal');
durs_trj_gen_dist   = fitdist(durs_trj_gen, 'Normal');
eval_durs_mp_dist   = fitdist(eval_durs_mp, 'Normal');
gen_durs_mp_dist    = fitdist(gen_durs_mp, 'Normal');
mod_durs_mp_dist    = fitdist(mod_durs_mp, 'Normal');
mutate_durs_mp_dist = fitdist(mutate_durs_mp, 'Normal');
error_corr_durs_eval_dist       = fitdist(error_corr_durs_eval, 'Normal');
error_corr_durs_no_eval_dist    = fitdist(error_corr_durs_no_eval, 'Normal');
