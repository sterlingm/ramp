points = importdata('full_trajectory.txt');

result = [];

% Get the indices of the points that start new control cycles
cc_starts = [];
thresh = 0.25;
for i=2:size(points,1)-1
    dist = sqrt( (points(i,1) - points(i-1,1))^2 + (points(i,2) - points(i-1,2))^2 );
    if dist > thresh
        cc_starts = [cc_starts; i];
    end
end

% Get each best trajectory for the control cycles
% Store the trajectories in a cell array
trajs = {1, size(cc_starts,1)};
last = 1;
for i=1:size(cc_starts,1)
    trajs{i} = points(last:cc_starts(i)-1, :);
    last = cc_starts(i);
end
% Add on last trajectory
trajs{ size(trajs,2) +1 } = points( cc_starts(size(cc_starts,1)):end, :);

cc_stops = [];
% For each trajectory
% Create an array of indices where the trajectory should stop for next CC
for i=1:size(trajs,2)-1
    % Get the starting point of the next trajectory
    p_next = trajs{1,i+1}(1,:);
    
    % For each point in trajectory
    % Find the point with min dist to p_next
    i_minDist = 1;
    for j=2:size( trajs{1,i}, 1)
        % Distance from point j on traj i to p_next
        dist = sqrt( (trajs{1,i}(j,1) - p_next(1))^2 + (trajs{1,i}(j,2) - p_next(2))^2 );
        
        % If dist < dist from point i_minDist on traj j to p_next
        if dist < sqrt( (trajs{1,i}(i_minDist,1) - p_next(1))^2 + (trajs{1,i}(i_minDist,2) - p_next(2))^2 )
            i_minDist = j;
        end
    end
    
    cc_stops = [cc_stops; i_minDist];
end
cc_stops = [cc_stops; size(trajs{1, size(trajs,2)},1)];

% For each trajectory
% Create cell array of only the parts that the robot moves on
trajs_partial = {};
for i=1:size(trajs,2)
    trajs_partial{i} = trajs{1,i}(1:cc_stops(i),:);
end

% Concatenate together all the partial trajectories
traj_final = [];
for i=1:size(trajs_partial,2)
    traj_final = [traj_final; trajs_partial{i}];
end