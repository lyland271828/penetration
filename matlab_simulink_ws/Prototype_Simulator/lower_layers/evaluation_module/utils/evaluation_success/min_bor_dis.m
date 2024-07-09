function nearestNeighborDistances = min_bor_dis(points)
    % 输入：
    % points - 一个n*2的矩阵，其中每一行代表一个点的x和y坐标
    % 输出：
    % nearestNeighborDistances - 一个n*1的向量，每个元素代表对应点到其最近邻居的距离

    % 获取点的数量
    n = size(points, 1);
    
    % 初始化最近邻距离向量
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
end