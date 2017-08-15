f_fe = 'feasible.txt';
f_re = 'reached_goal.txt';
f_ic_st = 'ic_stuck.txt';
f_ic_oc = 'ic_occurred.txt';
f_tl = 'time_left.txt';

fe = importdata(strcat('1/', f_fe));
fe = [fe; importdata(strcat('2/', f_fe))];
fe = [fe; importdata(strcat('3/', f_fe))];
fe = [fe; importdata(strcat('4/', f_fe))];
fe = [fe; importdata(strcat('5/', f_fe))];
fe = [fe; importdata(strcat('6/', f_fe))];
fe = [fe; importdata(strcat('7/', f_fe))];
fe = [fe; importdata(strcat('8/', f_fe))];

tl = importdata(strcat('1/', f_tl));
tl = [tl; importdata(strcat('2/', f_tl))];
tl = [tl; importdata(strcat('3/', f_tl))];
tl = [tl; importdata(strcat('4/', f_tl))];
tl = [tl; importdata(strcat('5/', f_tl))];
tl = [tl; importdata(strcat('6/', f_tl))];
tl = [tl; importdata(strcat('7/', f_tl))];
tl = [tl; importdata(strcat('8/', f_tl))];

re = importdata(strcat('1/', f_re));
re = [re; importdata(strcat('2/', f_re))];
re = [re; importdata(strcat('3/', f_re))];
re = [re; importdata(strcat('4/', f_re))];
re = [re; importdata(strcat('5/', f_re))];
re = [re; importdata(strcat('6/', f_re))];
re = [re; importdata(strcat('7/', f_re))];
re = [re; importdata(strcat('8/', f_re))];

ic_st = importdata(strcat('1/', f_ic_st));
ic_st = [ic_st; importdata(strcat('2/', f_ic_st))];
ic_st = [ic_st; importdata(strcat('3/', f_ic_st))];
ic_st = [ic_st; importdata(strcat('4/', f_ic_st))];
ic_st = [ic_st; importdata(strcat('5/', f_ic_st))];
ic_st = [ic_st; importdata(strcat('6/', f_ic_st))];
ic_st = [ic_st; importdata(strcat('7/', f_ic_st))];
ic_st = [ic_st; importdata(strcat('8/', f_ic_st))];


ic_oc = importdata(strcat('1/', f_ic_oc));
ic_oc = [ic_oc; importdata(strcat('2/', f_ic_oc))];
ic_oc = [ic_oc; importdata(strcat('3/', f_ic_oc))];
ic_oc = [ic_oc; importdata(strcat('4/', f_ic_oc))];
ic_oc = [ic_oc; importdata(strcat('5/', f_ic_oc))];
ic_oc = [ic_oc; importdata(strcat('6/', f_ic_oc))];
ic_oc = [ic_oc; importdata(strcat('7/', f_ic_oc))];
ic_oc = [ic_oc; importdata(strcat('8/', f_ic_oc))];
