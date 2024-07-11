function group_n = group_number(points)
%GROUP_NUMBER 计算形成集群的个数
    % 输入：
    % points - 一个n*2的矩阵，其中每一行代表一个点的x和y坐标
    % 输出：
    % group_n - 形成集群的个数
    
    % 获取点的数量
    n = size(points, 1);

    nearestNeighborDistances =min_bor_dis(points);
    % disp('最近邻间距离：')
    % disp(nearestNeighborDistances);
    
    
    % 计算均值和标准差
    mean_distance = mean(nearestNeighborDistances);
    std_distance = std(nearestNeighborDistances);
    
    % 计算正态分布范围
    upper_bound = mean_distance + 2 * std_distance;
    
    % 根据正态分布范围确定离群值
    outliers = (nearestNeighborDistances > upper_bound);
    
    % 输出离群值
    % disp('离群值：');
    % disp(nearestNeighborDistances(outliers));
    
    %形成集群的个数
    group_n=n-length(nearestNeighborDistances(outliers));
end

