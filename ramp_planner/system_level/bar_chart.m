

% Imminent Collision occurred
y = [ic_oc_mean; 1 - ic_oc_mean];
bar(y, 0.2);
x = {'IC Occurred'; 'No IC'};
y_tick = {'0'; '10'; '20'; '30'; '40'; '50'; '60'; '70'};
ylabel('% of test cases');
set(gca, 'XTicklabel', x);
set(gca, 'YTicklabel', y_tick);
set(gca, 'FontSize', 8);
set(gcf, 'Position', [10 10 250 250]);