function [pose, traj, flag] = mpc_plan(start, goal, varargin)

% @file: mpc_plan.m
% @breif: Model Predicted Control (MPC) motion planning
% @author: Winter
% @update: 2023.2.7
    p = inputParser;           
    addParameter(p, 'path', "none");
    addParameter(p, 'map', "none");
    parse(p, varargin{:});

    if isstring(p.Results.path) || isstring(p.Results.map)
        exception = MException('MyErr:InvalidInput', 'parameter `path` or `map` must be set.');
        throw(exception);
    end
    
    % path
    path = flipud(p.Results.path);
    path = path_interpolation(path, 5);
    
    % initial robotic state
    robot.x = start(1);
    robot.y = start(2);
    robot.theta = start(3);
    robot.v = 0;
    robot.w = 0;
    
    % common parameters
    param.dt = 0.1;%每次控制更新的时间间隔
    param.max_iteration = 2000;%最大迭代次数
    param.goal_dist_tol = 1.0;%目标距离容差，当车辆与目标点的距离小于这个容差时，认为车辆已经到达目标点
    param.rotate_tol = 0.5;%旋转容差，当车辆的朝向与目标朝向的偏差小于这个容差时，认为车辆的朝向已经达到要求。
    param.lookahead_time = 1.0;%控制器向前预测的时间范围，用于确定前瞻距离
    param.min_lookahead_dist = 1.0;%最小前瞻距离
    param.max_lookahead_dist = 2.5;%最大前瞻距离
    param.max_v_inc = 0.5;%最大线速度增量
    param.max_v = 1.0;
    param.min_v = 0.0;
    param.max_w_inc = pi / 2;%最大角速度增量
    param.max_w = pi / 2;
    param.min_w = 0.0;
    
    % MPC parameters
    param.Q = diag([1, 1, 1]);%状态权重矩阵
    param.R = diag([2, 2]);%控制输入权重矩阵
    param.p = 12;%预测时域长度。表示 MPC 算法向前预测的步数
    param.m = 8;%控制时域长度。表示 MPC 算法在每个控制周期内优化的控制输入步数
    param.u_min = [param.min_v; param.min_w];
    param.u_max = [param.max_v; param.max_w];
    param.du_min = [param.min_v; -param.max_w_inc];
    param.du_max = [param.max_v_inc; param.max_w_inc];
    
    % return value
    flag = false;
    pose = [];
    traj = [];%轨迹
    
    % main loop
    iter = 0;
    u_p = [0, 0];%先前的控制误差
    while iter < param.max_iteration
        iter = iter + 1;
        
        % break until goal reached
        if shouldRotateToGoal([robot.x, robot.y], goal, param)
            flag = true;
            break;
        end
        
        % get the particular point on the path at the lookahead distance
        %获取路径上前瞻距离处的特定点
        [lookahead_pt, theta_trj, kappa] = getLookaheadPoint(robot, path, param);
        
        % calculate velocity command
        e_theta = regularizeAngle(robot.theta - goal(3)) / 10;
        if shouldRotateToGoal([robot.x, robot.y], goal, param)%是否需要旋转到目标点
            if ~shouldRotateToPath(abs(e_theta), 0.0, param)
                u = [0, 0];
            else
                u = [0, angularRegularization(robot, e_theta / param.dt, param)];%将线速度设为 0，角速度经过规范化处理后赋值给 u，表示机器人只进行旋转运动
            end
        else
            e_theta = regularizeAngle( ...
                atan2(real(lookahead_pt(2)) - robot.y, real(lookahead_pt(1)) - robot.x) - robot.theta ...
            ) / 10;%从机器人当前位置到前瞻点的方向角度，减去机器人当前的朝向角度 robot.theta，并进行角度规范化和除以 10 的操作
            if shouldRotateToPath(abs(e_theta), pi / 4, param)
                u = [0, angularRegularization(robot, e_theta / param.dt, param)];
            else%不需要旋转到路径方向，使用模型预测控制（MPC）计算控制指令
                % current state
                s = [robot.x, robot.y, robot.theta];
                % desired state
                s_d = [real(lookahead_pt), theta_trj];
                % refered input 参考输入
                u_r = [robot.v, theta_trj - robot.theta];
                % control
                [u, u_p] = mpcControl(s, s_d, u_r, u_p, robot, param);
            end
        end
        
        % input into robotic kinematic
        robot = f(robot, u, param.dt);
        pose = [pose; robot.x, robot.y, robot.theta];
    end
end

%%
function robot = f(robot, u, dt)
    % robotic kinematic
    F = [ 1 0 0 0 0
             0 1 0 0 0
             0 0 1 0 0
             0 0 0 0 0
             0 0 0 0 0];
 
    B = [dt * cos(robot.theta) 0
            dt * sin(robot.theta)  0
            0                                dt
            1                                 0
            0                                 1];
 
    x = [robot.x; robot.y; robot.theta; robot.v; robot.w];
    x_star = F * x + B * u';
    robot.x = x_star(1); robot.y = x_star(2); robot.theta = x_star(3);
    robot.v = x_star(4); robot.w = x_star(5);
end

function theta = regularizeAngle(angle)
    theta = angle - 2.0 * pi * floor((angle + pi) / (2.0 * pi));
end
%比较机器人当前位置与目标位置的欧几里得距离和目标距离容差，来判断机器人是否需要执行旋转操作以到达目标位姿。如果距离小于容差，返回 true
function flag = shouldRotateToGoal(cur, goal, param)
    %{
    Whether to reach the target pose through rotation operation

    Parameters
    ----------
    cur: tuple
        current pose of robot
    goal: tuple
        goal pose of robot

    Return
    ----------
    flag: bool
        true if robot should perform rotation
    %}
    flag = hypot(cur(1) - goal(1), cur(2) - goal(2)) < param.goal_dist_tol;
end
%判断机器人是否需要通过旋转操作来纠正其跟踪路径，即判断机器人当前的朝向与路径期望朝向之间的角度偏差是否超出了可容忍的范围
function flag = shouldRotateToPath(angle_to_path, tol, param)
    %{
    Whether to correct the tracking path with rotation operation

    Parameters
    ----------
    angle_to_path: float 
        the angle deviation
    tol: float[None]
        the angle deviation tolerence

    Return
    ----------
    flag: bool
        true if robot should perform rotation
    %}
    if tol == 0.0
        flag = angle_to_path > param.rotate_tol;
    else
        flag = angle_to_path > tol;
    end
end
%对机器人的角速度进行正则化处理，确保角速度及其变化量在设定的允许范围内
function w = angularRegularization(robot, w_d, param)
    %{
    Angular velocity regularization

    Parameters
    ----------
    w_d: float
        reference angular velocity input

    Return
    ----------
    w: float
        control angular velocity output
    %}
    w_inc = w_d - robot.w;
    if abs(w_inc) > param.max_w_inc
        w_inc =param.max_w_inc * sign(w_inc);
    end
    w = robot.w + w_inc;

    if abs(w) > param.max_w
        w = param.max_w * sign(w);
    end
    if abs(w) < param.min_w
        w = param.min_w * sign(w)  ;
    end
end

function v = linearRegularization(robot, v_d, param)
    %{
    Linear velocity regularization

    Parameters
    ----------
    v_d: float
        reference velocity input

    Return
    ----------
    v: float
        control velocity output
    %}
    v_inc = v_d - robot.v;
    if abs(v_inc) > param.max_v_inc
        v_inc = param.max_v_inc * sign(v_inc);
    end
    v = robot.v + v_inc;

    if abs(v) > param.max_v
        v = param.max_v * sign(v);
    end
    if abs(v) < param.min_v
        v = param.min_v * sign(v);
    end
end
%计算机器人的前瞻距离
function d = getLookaheadDistance(robot, param)
    d = robot.v * param.lookahead_time;
    if d < param.min_lookahead_dist
        d = param.min_lookahead_dist;
    end
    if d > param.max_lookahead_dist
        d = param.max_lookahead_dist;
    end
end

function [pt, theta, kappa] = getLookaheadPoint(robot, path, param)
    %{
    Find the point on the path that is exactly the lookahead distance away from the robot

    Return
    ----------
    lookahead_pt: tuple
        lookahead point
    theta: float
        the angle on trajectory
    kappa: float
        the curvature on trajectory
    %}

    % Find the first pose which is at a distance greater than the lookahead distance、
    %找到第一个距离机器人大于等于前瞻距离的点
    dist_to_robot = [];
    [pts_num, ~] = size(path);% 使用 size 函数获取路径 `path` 的大小，`pts_num` 表示路径上点的数量，`~` 表示忽略路径点的维度信息（因为这里只关心点的数量）
    for i=1:pts_num
        % 使用 hypot 函数计算路径上第 `i` 个点到机器人当前位置的欧几里得距离
        dist_to_robot(end + 1) = hypot(path(i, 1) - robot.x, path(i, 2) - robot.y);
    end
    %%先找到路径上距离机器人最近的点，然后根据前瞻距离，从该最近点开始向后搜索，找到第一个距离机器人大于等于前瞻距离的点。
    [~, idx_closest] = min(dist_to_robot);
    idx_goal = pts_num - 1; idx_prev = idx_goal - 1;%初始化目标点和前一个点的索引
    
    lookahead_dist = getLookaheadDistance(robot, param);
    for i=idx_closest:pts_num
        if hypot(path(i, 1) - robot.x, path(i, 2) - robot.y) >= lookahead_dist
            idx_goal = i;
            break;
        end
    end
    if idx_goal == pts_num - 1
        % If the no pose is not far enough, take the last pose
        %之前的搜索中没有找到距离机器人大于等于前瞻距离的点，可能路径上所有点距离机器人都不够远 将路径上倒数第二个点作为前瞻点
        pt = [path(idx_goal, 1), path(idx_goal, 2)];
    else
        if idx_goal == 1
            idx_goal = idx_goal + 1;
        end
        % find the point on the line segment between the two poses
        % that is exactly the lookahead distance away from the robot pose (the origin)
        % This can be found with a closed form for the intersection of a segment and a circle
        %找到路径上距离机器人当前位置恰好为前瞻距离（lookahead_dist）的点，并计算该点所在路径线段的切线角度
        idx_prev = idx_goal - 1;
        px = path(idx_prev, 1); py = path(idx_prev, 2);
        gx = path(idx_goal, 1); gy = path(idx_goal, 2);
        
        % transform to the robot frame so that the circle centers at (0,0)
        %将线段的两个端点坐标从全局坐标系转换到以机器人位置为原点的局部坐标系
        prev_p = [px - robot.x, py - robot.y];
        goal_p = [gx - robot.x, gy - robot.y];
        i_points = circleSegmentIntersection(prev_p, goal_p, lookahead_dist);%计算线段与圆的交点
        pt = [i_points(1, 1) + robot.x, i_points(1, 2) + robot.y];%全局坐标系下的前瞻点坐标
    end

    % calculate the angle on trajectory
    %计算路径上 idx_prev 到 idx_goal 这两点所构成线段的切线角度
    theta = atan2(path(idx_goal, 2) - path(idx_prev, 2), path(idx_goal, 1) - path(idx_prev, 1));

    % calculate the curvature on trajectory 曲率
    if idx_goal == 2
        idx_goal = idx_goal + 1;
    end
    idx_prev = idx_goal - 1;
    idx_pprev = idx_prev - 1;
    %计算由这三个相邻点构成的三角形的三条边长
    a = hypot(path(idx_prev, 1) - path(idx_goal, 1), path(idx_prev, 2) - path(idx_goal, 2));
    b = hypot(path(idx_pprev, 1) - path(idx_goal, 1), path(idx_pprev, 2) - path(idx_goal, 2));
    c = hypot(path(idx_pprev, 1) - path(idx_prev, 1), path(idx_pprev, 2) - path(idx_prev, 2));
    cosB = (a * a + c * c - b * b) / (2 * a * c);
    sinB = sin(acos(cosB));
    %计算向量叉积来确定曲线的弯曲方向。叉积的正负可以表示曲线是向左弯曲还是向右弯曲
    cross = (path(idx_prev, 1) - path(idx_pprev, 1)) * ...
            (path(idx_goal, 2) - path(idx_pprev, 2)) - ...
            (path(idx_prev, 2) - path(idx_pprev, 2)) * ...
            (path(idx_goal, 1) - path(idx_pprev, 1));
    kappa = 2 * sinB / b *sign(cross); %曲率kappa= b/2sinB,sign(cross) 是为了给曲率加上方向信息，正曲率表示曲线向左弯曲，负曲率表示曲线向右弯曲。​
end

function i_points = circleSegmentIntersection(p1, p2, r)
    x1 = p1(1); x2 = p2(1);
    y1 = p1(2); y2 = p2(2);

    dx = x2 - x1; dy = y2 - y1;
    dr2 = dx * dx + dy * dy;%线段长度的平方
    D = x1 * y2 - x2 * y1;

    % the first element is the point within segment
    d1 = x1 * x1 + y1 * y1;
    d2 = x2 * x2 + y2 * y2;
    dd = d2 - d1;

    delta = sqrt(r * r * dr2 - D * D);%delta 的值决定了线段与圆的相交情况
    if delta >= 0
        if delta == 0%线段与圆相切
            i_points = [D * dy / dr2, -D * dx / dr2];
        else%线段与圆有两个交点
            i_points = [
                 (D * dy + sign(dd) * dx * delta) / dr2, ...
                (-D * dx + sign(dd) * dy * delta) / dr2; ...
                 (D * dy - sign(dd) * dx * delta) / dr2, ...
                 (-D * dx - sign(dd) * dy * delta) / dr2
            ];%sign(dd) 函数用于返回 dd 的符号（正为 1，负为 -1），用于确定交点的具体位置。
        end
    else
        i_points = [];
    end
end

function path_new = path_interpolation(path, n)
    for i=1:n
        path_new = path;
        path_inter = path_new(1:end - 1, :) + diff(path_new) / 2;
        path = zeros(length(path_new) + length(path_inter), 2);
        path(1:2:end, :) = path_new;
        path(2:2:end, :) = path_inter;
    end
end

function [u, u_p_new] = mpcControl(s, s_d, u_r, u_p, robot, param)
    %{
    Execute MPC control process.

    Parameters
    ----------
    s: tuple
        current state
    s_d: tuple
        desired state
    u_r: tuple
        refered control
    u_p: tuple
        previous control error

    Return
    ----------
    u: np.ndarray
        control vector
    %}
    dim_u = 2; dim_x = 3;
    dt = param.dt;

    % state vector (5 x 1)
    x = [s - s_d, u_p]';
    
    % original state matrix
    A = eye(3);
    A(1, 3) = -u_r(1) * sin(s_d(3)) * dt;
    A(2, 3) = u_r(1) * cos(s_d(3)) * dt;

    % original control matrix
    B = zeros(3, 2);
    B(1, 1) = cos(s_d(3)) * dt;
    B(2, 1) = sin(s_d(3)) * dt;
    B(3, 2) = dt;

    % discrete iteration Ricatti equation
    P = param.Q;

    % state matrix (5 x 5)
    A = [A, B; zeros(dim_u, dim_x), eye(dim_u)];

    % control matrix (5 x 2)
    B = [B; eye(dim_u)];

    % output matrix (3 x 5)
    C = [eye(dim_x), zeros(dim_x, dim_u)];
    
    % mpc state matrix (3p x 5)
    S_x_cell = cell(param.p, 1);%预测时域长度。表示 MPC 算法向前预测的步数
    for i=1:param.p
        S_x_cell{i, 1} = C * A ^ i;
    end
    S_x = cell2mat(S_x_cell);
    
    % mpc control matrix (3p x 2m)
    S_u_cell = cell(param.p, param.m);
    for i = 1:param.p
        for j = 1:param.m
            if j <= i
                S_u_cell{i, j} = C * A ^ (i - j) * B;
            else
                S_u_cell{i, j} = zeros(dim_x, dim_u);
            end
        end
    end
    S_u = cell2mat(S_u_cell);
    
    % optimization
    Yr = zeros(3 * param.p, 1);                  % (3p x 1)
    Q = kron(eye(param.p), param.Q);    % (3p x 3p)
    R = kron(eye(param.m), param.R);    % (2m x 2m)
    H = S_u' * Q * S_u + R;                        % (2m x 2m)
    g = S_u' * Q * (S_x * x - Yr);                 % (2m x 1)
    
    % constriants
    A_I = kron(tril(ones(param.m, param.m)), diag([1, 1]));
    U_min = kron(ones(param.m, 1), param.u_min);
    U_max = kron(ones(param.m, 1), param.u_max);
    U_k_1 = kron(ones(param.m, 1), u_p');
    
    % boundary
    dU_min = kron(ones(param.m, 1), param.du_min);
    dU_max = kron(ones(param.m, 1), param.du_max);
    
    % solve
    options = optimoptions('quadprog', 'MaxIterations', 100, 'TolFun', 1e-16, 'Display','off');
    dU_opt = quadprog(H, g, [-A_I; A_I], [-U_min + U_k_1; U_max - U_k_1], [], [], dU_min, dU_max, [], options);
    
    % first element
    du = [dU_opt(1), dU_opt(2)];

    % real control
    u = du + u_p + u_r;
    
    u = [linearRegularization(robot, u(1), param), angularRegularization(robot, u(2), param)];
    u_p_new = u - u_r;
end
