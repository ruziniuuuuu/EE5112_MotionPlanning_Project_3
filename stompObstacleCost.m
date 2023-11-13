function cost = stompObstacleCost(sphere_centers, radius, voxel_world, vel)

safety_margin = 0.05; % the safety margin distance, unit: meter
cost = 0;
% signed distance function of the world
voxel_world_sEDT = voxel_world.sEDT;
world_size = voxel_world.world_size;
% calculate which voxels the sphere centers are in. idx is the grid xyz subscripts
% in the voxel world.
env_corner = voxel_world.Env_size(1,:); % [xmin, ymin, zmin] of the metric world
env_corner_vec = repmat(env_corner,length(sphere_centers),1); % copy it to be consistent with the size of sphere_centers
idx = ceil((sphere_centers-env_corner_vec)./voxel_world.voxel_size);

%% TODO: complete the following code according to Eq (13) in the STOMP conference paper.
try
    [nSpheres, ~] = size(sphere_centers);
    epsilon = mvnrnd(zeros(nSpheres, 1), 1);
    d = zeros(nSpheres, 1);
    for k = 1:nSpheres
        d(k) = voxel_world_sEDT(idx(k, 1), idx(k, 2), idx(k, 3));
    end
    cost_array = max(epsilon + radius - d(k), 0).*vel;
    cost = sum(cost_array);
catch  % for debugging
    idx = ceil((sphere_centers-env_corner_vec)./voxel_world.voxel_size);
end