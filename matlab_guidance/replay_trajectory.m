% replay_trajectory.m
% 从 CSV 飞行数据重建弹道与制导律回放
% 传感器: IMU (四元数+陀螺仪+加速度计) + 相机 (像素坐标+目标面积)
%
% 输入: flight_data.csv (由 K230 记录的飞行数据)
% 输出: 轨迹可视化、制导指令对比、LOS 分析

clc; clear; close all;

%% ===== 配置参数 =====
CSV_FILE = 'flight_data.csv';       % 数据文件路径
LAUNCH_PITCH_DEG = 35.0;            % 发射俯仰角 (度)
LAUNCH_SPEED = 17.5;                % 发射初速度 (m/s)
DART_POS_INIT = [0; 0; 0];         % 发射位置
GRAVITY = 9.81;                     % 重力加速度
TARGET_REAL_AREA = 0.0225;          % 目标真实面积 m² (15cm × 15cm)

% 制导律配置 (与仿真一致)
config.nav_ratio = 5.5;
config.camera_matrix = [500, 0, 320, 0, 500, 240, 0, 0, 1];
config.filter_alpha = 0.8;
config.max_accel_yaw = 20.0;
config.max_accel_pitch = 20.0;

%% ===== 1. 读取 CSV 数据 =====
fprintf('读取数据: %s\n', CSV_FILE);
try
    data = readtable(CSV_FILE);
catch
    error('无法读取 %s，请确认文件路径和格式正确。', CSV_FILE);
end

N = height(data);
fprintf('共 %d 帧\n', N);

% 提取时间戳
timestamp_ms = data.timestamp_ms;
dt_frames = double(timestamp_ms(2:end) - timestamp_ms(1:end-1)) / 1000.0;
mean_dt = mean(dt_frames);
fprintf('平均帧间隔: %.1f ms (%.1f Hz)\n', mean_dt*1000, 1/mean_dt);

% 提取相机数据
pixel_dx = data.pixel_dx;
pixel_dy = data.pixel_dy;
target_area = data.target_area;

% 提取 IMU 数据
qw = data.qw; qx = data.qx; qy = data.qy; qz = data.qz;
gyro_x = data.gyro_x; gyro_y = data.gyro_y; gyro_z = data.gyro_z;
accel_x = data.accel_x; accel_y = data.accel_y; accel_z = data.accel_z;

% 提取制导指令（如存在）
has_accel_cmd = all(ismember({'accel_cmd_pitch', 'accel_cmd_yaw'}, data.Properties.VariableNames));
if has_accel_cmd
    accel_cmd_pitch_rec = data.accel_cmd_pitch;
    accel_cmd_yaw_rec = data.accel_cmd_yaw;
end

% 提取飞行阶段（如存在）
has_phase = ismember('phase', data.Properties.VariableNames);
if has_phase
    phase = data.phase;
end

%% ===== 2. IMU 加速度积分重建弹道 =====
fprintf('重建弹道...\n');

% 预分配
dart_pos = zeros(3, N);
dart_vel = zeros(3, N);
dart_pos(:,1) = DART_POS_INIT;
dart_vel(:,1) = [LAUNCH_SPEED * cosd(LAUNCH_PITCH_DEG); 0; -LAUNCH_SPEED * sind(LAUNCH_PITCH_DEG)];

for i = 1:N-1
    dt = dt_frames(i);
    if dt <= 0 || dt > 0.5
        dt = mean_dt;  % 异常帧用平均帧间隔替代
    end

    % 归一化四元数
    nq = norm([qw(i), qx(i), qy(i), qz(i)]);
    q_norm = [qw(i), qx(i), qy(i), qz(i)] / max(nq, eps);

    % 弹体系加速度 → 惯性系
    a_body = [accel_x(i); accel_y(i); accel_z(i)];
    a_inertial = quat_rotate(q_norm, a_body);

    % 扣除重力 (惯性系 Z 向上取正, 重力向下)
    a_inertial = a_inertial - [0; 0; GRAVITY];

    % 欧拉积分
    dart_vel(:,i+1) = dart_vel(:,i) + a_inertial * dt;
    dart_pos(:,i+1) = dart_pos(:,i) + dart_vel(:,i) * dt;
end

%% ===== 3. 制导律回放验证 =====
fprintf('回放制导律...\n');

guide_sys = Guidance();
% 用首帧姿态作为参考四元数
q0_norm = [qw(1), qx(1), qy(1), qz(1)] / max(norm([qw(1), qx(1), qy(1), qz(1)]), eps);
guide_sys.guidance_init(config, q0_norm(1), q0_norm(2), q0_norm(3), q0_norm(4));
guide_sys.fire();

accel_pitch_replay = zeros(1, N);
accel_yaw_replay = zeros(1, N);
los_yaw_replay = zeros(1, N);
los_pitch_replay = zeros(1, N);
rate_yaw_replay = zeros(1, N);
rate_pitch_replay = zeros(1, N);
valid_replay = false(1, N);

has_locked = false;
first_lock_frame = 0;

for i = 1:N
    % 制导只在下降段 + 目标有效时启动
    if has_phase
        in_guidance = (phase(i) == 2);
    else
        % 无阶段信息时: 下降段判断 (vel_z > 0 即向下)
        in_guidance = (dart_vel(3,i) > 0) && (abs(pixel_dx(i)) > 0 || abs(pixel_dy(i)) > 0);
    end

    if in_guidance
        if ~has_locked
            has_locked = true;
            first_lock_frame = i;
            % 伪造前帧时间戳和 LOS 角度避免首帧突变
            fake_dt_ms = uint32(mean_dt * 1000);
            guide_sys.pre_timestamp = uint32(timestamp_ms(i)) - fake_dt_ms;
            % 用像素坐标估算初始 LOS 角
            fx = config.camera_matrix(1); fy = config.camera_matrix(5);
            guide_sys.prev_los_angle_yaw = atan2(pixel_dx(i), fx);
            guide_sys.prev_los_angle_pitch = atan2(-pixel_dy(i), fy);
        end

        % 估计接近速度: 取重建速度标量 (近似)
        closing_vel_est = max(norm(dart_vel(:,i)), 5.0);

        guide_sys.guidance_update(pixel_dx(i), pixel_dy(i), ...
            qw(i), qx(i), qy(i), qz(i), ...
            closing_vel_est, uint32(timestamp_ms(i)));
        out = guide_sys.guidance_calculate();

        if isstruct(out) && out.valid
            accel_pitch_replay(i) = out.accel_pitch;
            accel_yaw_replay(i) = out.accel_yaw;
            valid_replay(i) = true;
        end
        los_yaw_replay(i) = guide_sys.prev_los_angle_yaw;
        los_pitch_replay(i) = guide_sys.prev_los_angle_pitch;
        rate_yaw_replay(i) = guide_sys.los_angle_yaw_velocity;
        rate_pitch_replay(i) = guide_sys.los_angle_pitch_velocity;
    else
        has_locked = false;
    end
end

fprintf('制导帧数: %d / %d\n', sum(valid_replay), N);

%% ===== 4. 指令对比分析 =====
if has_accel_cmd
    valid_idx = find(valid_replay);
    if ~isempty(valid_idx)
        err_pitch = accel_pitch_replay(valid_idx) - accel_cmd_pitch_rec(valid_idx)';
        err_yaw = accel_yaw_replay(valid_idx) - accel_cmd_yaw_rec(valid_idx)';
        rmse_pitch = sqrt(mean(err_pitch.^2));
        rmse_yaw = sqrt(mean(err_yaw.^2));
        fprintf('回放 RMSE - Pitch: %.3f m/s², Yaw: %.3f m/s²\n', rmse_pitch, rmse_yaw);
    end
end

%% ===== 5. 可视化 =====
time_axis = double(timestamp_ms - timestamp_ms(1)) / 1000.0;

% --- 图1: 弹道重建 ---
figure('Name', '弹道重建', 'Position', [50, 50, 1200, 800]);

subplot(2,3,[1 2 3]);
plot3(dart_pos(1,:), dart_pos(2,:), -dart_pos(3,:), 'b-', 'LineWidth', 1.5); hold on;
scatter3(dart_pos(1,1), dart_pos(2,1), -dart_pos(3,1), 80, 'g', 'filled');  % 起点
scatter3(dart_pos(1,end), dart_pos(2,end), -dart_pos(3,end), 80, 'r', 'filled'); % 终点
grid on; axis equal;
title('3D 重建弹道 (IMU 加速度积分)');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m, 向上为正)');
legend('轨迹', '起点', '终点', 'Location', 'best');

subplot(2,3,4);
plot(time_axis, dart_vel(1,:), 'r', 'LineWidth', 1.2); hold on;
plot(time_axis, dart_vel(2,:), 'g', 'LineWidth', 1.2);
plot(time_axis, dart_vel(3,:), 'b', 'LineWidth', 1.2);
grid on; title('重建速度 (惯性系)');
xlabel('时间 (s)'); ylabel('速度 (m/s)');
legend('V_x', 'V_y', 'V_z');

subplot(2,3,5);
plot(time_axis, dart_pos(3,:), 'b', 'LineWidth', 1.5); grid on;
title('高度变化');
xlabel('时间 (s)'); ylabel('Z (m)');

subplot(2,3,6);
plot(time_axis, sqrt(sum(dart_vel.^2, 1)), 'k', 'LineWidth', 1.5); grid on;
title('合速度');
xlabel('时间 (s)'); ylabel('速度 (m/s)');

% --- 图2: 制导律回放 vs 记录 ---
figure('Name', '制导指令对比', 'Position', [150, 100, 1200, 600]);

subplot(2,2,1);
plot(time_axis, accel_pitch_replay, 'r-', 'LineWidth', 1.5); hold on;
if has_accel_cmd
    plot(time_axis, accel_cmd_pitch_rec, 'k--', 'LineWidth', 1.0);
    legend('回放', '记录');
else
    legend('回放');
end
grid on; title('俯仰加速度指令');
xlabel('时间 (s)'); ylabel('a_{pitch} (m/s²)');

subplot(2,2,2);
plot(time_axis, accel_yaw_replay, 'b-', 'LineWidth', 1.5); hold on;
if has_accel_cmd
    plot(time_axis, accel_cmd_yaw_rec, 'k--', 'LineWidth', 1.0);
    legend('回放', '记录');
else
    legend('回放');
end
grid on; title('偏航加速度指令');
xlabel('时间 (s)'); ylabel('a_{yaw} (m/s²)');

subplot(2,2,3);
plot(time_axis, pixel_dx, 'r.', 'MarkerSize', 4); hold on;
plot(time_axis, pixel_dy, 'b.', 'MarkerSize', 4);
grid on; title('像素差 (记录)');
xlabel('时间 (s)'); ylabel('pixel');
legend('dx (偏航)', 'dy (俯仰)');

subplot(2,2,4);
if has_accel_cmd
    scatter(accel_cmd_pitch_rec(valid_replay), accel_pitch_replay(valid_replay), 10, 'r'); hold on;
    scatter(accel_cmd_yaw_rec(valid_replay), accel_yaw_replay(valid_replay), 10, 'b');
    lims = [min([xlim ylim]), max([xlim ylim])];
    plot(lims, lims, 'k--');
    grid on; title('回放 vs 记录 (散点)');
    xlabel('记录指令 (m/s²)'); ylabel('回放指令 (m/s²)');
    legend('Pitch', 'Yaw', 'y=x', 'Location', 'best');
    axis equal;
else
    text(0.5, 0.5, '无记录指令对比', 'Units', 'normalized', ...
        'HorizontalAlignment', 'center', 'FontSize', 14);
end

% --- 图3: 制导律内部状态 ---
figure('Name', '制导律内部状态', 'Position', [250, 150, 900, 700]);

subplot(4,1,1);
plot(time_axis, los_yaw_replay, 'b', 'LineWidth', 1.5); grid on;
title('视线偏航角 (LOS Yaw)'); ylabel('rad');

subplot(4,1,2);
plot(time_axis, rate_yaw_replay, 'r', 'LineWidth', 1.5); grid on;
title('视线偏航角速度 (LOS Rate Yaw)'); ylabel('rad/s');

subplot(4,1,3);
plot(time_axis, los_pitch_replay, 'b', 'LineWidth', 1.5); grid on;
title('视线俯仰角 (LOS Pitch)'); ylabel('rad');

subplot(4,1,4);
plot(time_axis, rate_pitch_replay, 'r', 'LineWidth', 1.5); grid on;
title('视线俯仰角速度 (LOS Rate Pitch)'); ylabel('rad/s');
xlabel('时间 (s)');

% --- 图4: 像素/目标面积 ---
figure('Name', '传感器原始数据', 'Position', [350, 200, 800, 500]);

subplot(2,1,1);
yyaxis left;
plot(time_axis, pixel_dx, 'r.', 'MarkerSize', 4); hold on;
plot(time_axis, pixel_dy, 'b.', 'MarkerSize', 4);
ylabel('像素差'); grid on;
yyaxis right;
plot(time_axis, target_area, 'k-', 'LineWidth', 1.0);
ylabel('目标面积 (pixel²)');
title('相机数据'); xlabel('时间 (s)');
legend('dx', 'dy', 'area');

subplot(2,1,2);
plot(time_axis, gyro_x, 'r', 'LineWidth', 1.0); hold on;
plot(time_axis, gyro_y, 'g', 'LineWidth', 1.0);
plot(time_axis, gyro_z, 'b', 'LineWidth', 1.0);
grid on; title('IMU 陀螺仪角速度');
xlabel('时间 (s)'); ylabel('rad/s');
legend('gyro_x (滚转)', 'gyro_y (俯仰)', 'gyro_z (偏航)');

fprintf('完成。\n');

%% ===== 辅助函数 =====
function q = euler_to_quat(yaw, pitch, roll)
    cy = cos(yaw * 0.5); sy = sin(yaw * 0.5);
    cp = cos(pitch * 0.5); sp = sin(pitch * 0.5);
    cr = cos(roll * 0.5); sr = sin(roll * 0.5);
    q = [cr*cp*cy + sr*sp*sy;
         sr*cp*cy - cr*sp*sy;
         cr*sp*cy + sr*cp*sy;
         cr*cp*sy - sr*sp*cy];
end

function v_rot = quat_rotate(q, v)
    if numel(v) < 3, v_rot = [0;0;0]; return; end
    q = q(:); v = v(:);
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    vx = v(1); vy = v(2); vz = v(3);
    cx1 = qy*vz - qz*vy; cy1 = qz*vx - qx*vz; cz1 = qx*vy - qy*vx;
    tx = cx1 + qw*vx; ty = cy1 + qw*vy; tz = cz1 + qw*vz;
    cx2 = qy*tz - qz*ty; cy2 = qz*tx - qx*tz; cz2 = qx*ty - qy*tx;
    v_rot = [vx + 2.0*cx2; vy + 2.0*cy2; vz + 2.0*cz2];
end
