fe_mean = mean(data_all_fe);
re_mean = mean(data_all_re);
tl_mean = mean(tl);
ic_st_mean = mean(data_all_ic_st);
ic_oc_mean = mean(ic_oc);

fe_pie_mean = fe_mean - re_mean;

infe_mean = 1 - fe_mean - ic_st_mean;

perc_values = {sprintf('%.0f%%', 100*re_mean); sprintf('%.0f%%', 100*fe_pie_mean); sprintf('%.0f%%', 100*infe_mean); sprintf('%.0f%%', 100*ic_st_mean)};

strs = {strcat('Goal: ', perc_values{1}) ; strcat('Feasible: ', perc_values{2}) ; strcat('Infeasible: ', perc_values{3}); strcat('I.C. (stuck): ', perc_values{4})};

pie_values = [re_mean, fe_pie_mean, infe_mean, ic_st_mean];

p = pie(pie_values, strs);
set(p(2:2:8),'FontSize',20);
set(gcf, 'Position', [10 10 500 500]);



% New pie chart - Time left
thresh = 10;
a = tl > thresh;
tl_past_thresh = sum(a);

a = (size(tl,1) - tl_past_thresh) / size(tl,1);
b = 1 - a;

perc_values = {sprintf('%.0f%%', 100*a); sprintf('%.0f%%', 100*b)};
strs = {strcat('T<10s: ', perc_values{1}) ; strcat('T>10s: ', perc_values{2})};

y = [a; b];

pie(y, strs);






% New pie chart - IC Occurred
a = ic_oc_mean;
b = 1 - a;

perc_values = {sprintf('%.0f%%', 100*a); sprintf('%.0f%%', 100*b)};
strs = {strcat('IC Occurred: ', perc_values{1}) ; strcat('', perc_values{2})};

y = [a; b];

pie(y);