function success_rate = success_define(states,r_coll, map3d_faces, map3d_struct,target)
    % 输入：
    % points - 一个n*2的矩阵，其中每一行代表一个点的x和y坐标，终点状态
    % mincoll 个体间最小的避碰距离
    % target  终点坐标
    % 输出：
    % success_rate - 成功率
        % 获取点的数量


    position = states(1:2,:);
    points = position';
    %velocity = states(4:5,:);
    n = size(points, 1);

%% 1，计算个体间的碰撞，coll
    % 初始化最近邻距离向量
    coll_success=zeros(n,1);  %为n行1列形式，碰撞为0；
    nearestNeighborDistances = zeros(n, 1);
    
    % 遍历每个点以计算到最近邻的距离
    for i = 1:n
        % 初始化最小距离为无穷大
        minDistance = inf;
        for j = 1:n
            % 跳过与自身的比较
            if i == j
                continue;
            end
            % 计算点i到点j的欧氏距离
            distance = sqrt((points(i, 1) - points(j, 1))^2 + (points(i, 2) - points(j, 2))^2);
            % 更新最小距离
            if distance < minDistance
                minDistance = distance;
            end
        end
        % 存储到最近邻的距离
        nearestNeighborDistances(i) = minDistance;
    end

    for i=1:n
        if nearestNeighborDistances(i)<r_coll
            coll_success(i,1) = 0;
        else
            coll_success(i,1) = 1;
        end
    end


%% 2, 计算与墙壁的碰撞，wall
    wall_success=zeros(n,1);  %为n行1列形式，碰撞为1；
    out_of_map = map_module_out_of_map(states(1:3,:),map3d_struct);
    col_map = map_module_collision_detection(states(1:3,:),map3d_faces,r_coll/2);
    outOfMap = out_of_map | col_map;
    wall_success = outOfMap';



%% 3, 计算是否离群，group,离群为0.
    group_success = zeros(n,1);
    % 计算均值和标准差
    mean_distance = mean(nearestNeighborDistances);
    std_distance = std(nearestNeighborDistances);
    
    % 计算正态分布范围
    upper_bound = mean_distance + 2 * std_distance;
    
    % 根据正态分布范围确定离群值
    for i = 1:n
        if nearestNeighborDistances(i) > upper_bound
            group_success(i) = 0;
        else
            group_success(i) = 1;
        end
    end

%% 4,计算是否到达终点
    arrive=zeros(n,1);
    for i =1:n
        if points(i,1)>target(1,1)-50
            arrive(i)=1;
        else
            arrive(i)=0;
        end
    end



    agent_success = coll_success & (~wall_success) & group_success & arrive;
    agent_sum = sum(agent_success);
    success_rate = agent_sum/n*100;

end

