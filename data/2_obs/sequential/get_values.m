function [data] = get_values(fstr)
enter_str = ['In get_values, fstr: ',fstr];
disp(enter_str);

for num=1:2
    numstr = num2str(num);
    fname = strcat(numstr, '/', fstr);
    disp(fname);
    if num == 1
        data = importdata(fname);
    else
        data = [data; importdata(fname)];
    end
end

disp('Exiting get_values');