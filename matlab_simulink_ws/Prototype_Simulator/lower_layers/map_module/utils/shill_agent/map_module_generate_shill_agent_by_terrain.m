function [pos_shill,vel_shill] = map_module_generate_shill_agent_by_terrain(state_i,terrain,terrain_params,r_w)
%MAP_MODULE_GENE 此处显示有关此函数的摘要
%   此处显示详细说明
% Local terrain to shill agents

if ~isempty(terrain)
    % r_w = 5;
    r_sub = floor((state_i(2)-terrain_params(2,1))/terrain_params(2,2));
    c_sub = floor((state_i(1)-terrain_params(1,1))/terrain_params(1,2));
    h_sub = floor((r_w/terrain_params(2,2)));
    w_sub = floor((r_w/terrain_params(1,2)));
    [h,w] = size(terrain);
    r_min = max(1,r_sub-h_sub);
    r_max = min(h,r_sub+h_sub);
    c_min = max(1,c_sub-w_sub);
    c_max = min(w,c_sub+w_sub);
    terrain_sub = terrain(r_min:r_max,c_min:c_max);
    [r_obs,c_obs] = find(terrain_sub>state_i(3));
    if ~isempty(r_obs)
        ind = sub2ind(size(terrain_sub),r_obs,c_obs);
        obs_height = terrain_sub(ind);
        r_obs = r_obs + r_min - 1;
        c_obs = c_obs + c_min - 1;
        temp_p_shill = [(c_obs'*terrain_params(1,2))+terrain_params(1,1)
            (r_obs'*terrain_params(2,2))+terrain_params(2,1)
            obs_height'];
        temp = [state_i(1:3) - temp_p_shill];
        vel_shill = [temp./vecnorm(temp)];
        pos_shill = [temp_p_shill];
    else
        pos_shill=[];
        vel_shill=[];
    end
else
    pos_shill=[];
    vel_shill=[];
end

end

