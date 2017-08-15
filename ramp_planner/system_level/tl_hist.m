% Remove smallest % of goal_reached from time_left

tl_sorted = sort(tl);

i_start = round(size(tl,1)*re_mean);

tl_stripped = tl_sorted(i_start:end);

hist(tl_stripped, 25);
xlabel('Seconds left in trajectory');