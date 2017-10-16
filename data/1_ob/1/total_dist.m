dist = 0;
for i=1:size(traj_final,1)-1
    dist = dist + sqrt( (traj_final(i,1) - traj_final(i+1,1))^2 + (traj_final(i,2) - traj_final(i+1,2))^2 );
end