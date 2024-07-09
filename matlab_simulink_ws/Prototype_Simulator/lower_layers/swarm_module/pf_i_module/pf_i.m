function [velDesired_id] = pf_i(id,N,position,target,obs,ijR,cR,obR,alpha_attij,alpha_rejij,alpha_rejob,alpha_atttar,v_max)
% 输入：智能体序号id；集群包含智能体个数N；智能体位置position(3*N)，目标区域位置target(3*1)；障碍物位置obs(3*N_obs)
%       智能体间吸引力参数alpha_attij；智能体间排斥力参数alpha_rejij；障碍物排斥力参数alpha_rejob；目标区域吸引力参数alpha_atttar；智能体最大速度参数v_max。
% 输出：智能体id的期望速度velDesired_id

%%

% global time_k; % 记录采样周期的个数
% global blood;
% global initialBlood

posId = position(:,id); % i智能体位置
posij = kron(ones(1,N),posId)-position; % ij智能体的相对位置

disij = sqrt(sum(posij.^2,1)); % ij智能体的相对距离(1*N)
disij(:,id) = inf;

lessThanijR = disij < ijR;
neighborIndexijR = find(lessThanijR); % 与i智能体距离相近的索引j(小于该值防止防撞)

disij(:,id) = 0;
greatThancR = disij > cR;
neighborIndexcR = find(greatThancR); % 与i智能体距离较远的索引j(大于该值相互吸引)

%% attracted by j
attID_j = zeros(3,1);
if ~isempty(neighborIndexcR)
    dis_attij = disij(neighborIndexcR);
    pos_attij = posij(:,neighborIndexcR);
    % attID_j = -alpha_attij * sum(pos_attij .* sqrt(dis_attij),2);
    attID_j = -alpha_attij * sum(sqrt(dis_attij) .* pos_attij ./ dis_attij,2);
end

%% rejected by j
rejID_j = zeros(3,1);
if ~isempty(neighborIndexijR)
    dis_rejij = disij(neighborIndexijR);
    pos_rejij = posij(:,neighborIndexijR);
    rejID_j = alpha_rejij * sum(pos_rejij ./ (dis_rejij-1),2);
end

%% rejected by obstacles
rejID_ob = zeros(3,1);
if ~isempty(obs)
    posi_ob = kron(ones(1,size(obs,2)),posId) - obs; % i智能体与障碍物之间的相对位置
    disi_ob = sqrt(sum(posi_ob.^2,1)); % i智能体到所有障碍物之间的距离

    lessThanobR = disi_ob < obR;
    neighborIndexobR = find(lessThanobR); % 与i智能体距离相近的障碍物索引j(小于该值防止防撞)

    if ~isempty(neighborIndexobR)
        dis_rejiob = disi_ob(neighborIndexobR);
        pos_rejiob = posi_ob(:,neighborIndexobR);
        rejID_ob = alpha_rejob * sum(pos_rejiob ./ (dis_rejiob-5),2);
    end
end
% target

%% attracted by target
posi_tar = posId - target;
disi_tar = sqrt(sum(posi_tar.^2,1));

% attID_tar = -alpha_atttar * posi_tar .* sqrt(disi_tar);
attID_tar = -alpha_atttar * sqrt(disi_tar) .* posi_tar ./ disi_tar;


%% desired vel
velDesired_id = attID_j + rejID_j + rejID_ob + attID_tar;

%% limit
vel_norm = norm(velDesired_id);
if vel_norm > v_max
    velDesired_id = velDesired_id./vel_norm * v_max;
end


%% 雷达位置
% pos_lidar = [-169;51;0];
% Rmax = 155;
% boold_min = 20;


% [initialBlood] = radar_model(id,N,position,pos_lidar,Rmax,boold_min);
% 
% if initialBlood(1,id) < boold_min
%     if posId(3) > 0 
%         velDesired_id = [0;0;-5];
%     else
%         velDesired_id = [0;0;0];
%     end
% end


%%



end