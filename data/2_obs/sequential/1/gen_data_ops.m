% General data operations
pc_freqs_dist = fitdist(pc_freqs, 'Normal');
cc_freqs_dist = fitdist(cc_freqs, 'Normal');


% Duration data operations
pc_durs_dist = fitdist(pc_durs, 'Normal');
sc_durs_dist = fitdist(sc_durs, 'Normal');
cc_durs_dist = fitdist(cc_durs, 'Normal');

trajec_durs_dist = fitdist(trajec_durs, 'Normal');
eval_durs_dist = fitdist(eval_durs, 'Normal');
mod_durs_dist = fitdist(mod_durs, 'Normal');
trajec_durs_nomp_dist = fitdist(durs_trj_gen, 'Normal');
eval_durs_nomp_dist = fitdist(durs_trj_eval, 'Normal');
mod_durs_nomp_dist = fitdist(durs_path_mods, 'Normal');
mutate_durs_dist = fitdist(mutate_durs, 'Normal');
error_correct_durs_eval_dist = fitdist(error_correct_durs_eval, 'Normal');
error_correct_durs_noeval_dist = fitdist(error_correct_durs_no_eval, 'Normal');


% Execution data
runtime;
switch_t_size_dist = fitdist(switch_t_size, 'Normal');
trajec_size_dist = fitdist(trajec_size, 'Normal');
time_in_ic;
motion_error_amount_dist = fitdist(motion_error_amount, 'Normal');
min_dist_obs_dist = fitdist(min_dist_obs, 'Normal');