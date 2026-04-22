% === 飞镖视觉比例导引 闭环仿真 
clc; clear all; close all;

%% 1. 物理环境与初始参数配置
target_init_pos = [25.0; 0.0; -0.5];  
target_pos = target_init_pos;         

launch_pitch_deg = 35.0;         
launch_yaw_deg = 0.0;
v0 = 17.5;                       
dart_pos = [0.0; 0.0; 0.0];      
dart_vel = [v0 * cosd(launch_pitch_deg); 0; -v0 * sind(launch_pitch_deg)]; 

cam_matrix = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0];
fov_x_half = atan(320.0 / 500.0); 
fov_y_half = atan(240.0 / 500.0); 

ENABLE_GRAVITY_COMPENSATION = false; 

%% 2. 初始化制导系统
config.nav_ratio = 5.5;             
config.camera_matrix = cam_matrix;  
config.filter_alpha = 0.8;          
config.max_accel_yaw = 20.0;        
config.max_accel_pitch = 20.0;      

guide_sys = Guidance();
init_q = euler_to_quat(launch_yaw_deg*pi/180, launch_pitch_deg*pi/180, 0);
guide_sys.guidance_init(config, init_q(1), init_q(2), init_q(3), init_q(4));
guide_sys.fire();

%% 3. 仿真循环
fps = 60;
dt = 1.0 / fps;           
N = ceil(4.0 / dt);

history_dart_pos = zeros(3, N); history_target_pos = zeros(3, N); 
history_dx_dy = NaN(2, N); history_accel = zeros(2, N); history_phase = zeros(1, N); 
history_miss_distance = zeros(1, N); 

% 目标尺寸：15cm × 15cm 正方形
TARGET_HALF = 0.075;  % 半边长
min_miss_dist = inf;  % 记录最小脱靶量

% === 新增：透视制导律内部变量 ===
debug_los_yaw = zeros(1, N);
debug_rate_yaw = zeros(1, N);
debug_los_pitch = zeros(1, N);
debug_rate_pitch = zeros(1, N);

current_time_ms = uint32(0); hit_flag = false; final_miss_dist = NaN; has_locked_on = false;

fprintf('🚀 飞镖发射！等待下落段捕捉目标...\n');

for i = 1:N
    current_time_s = double(current_time_ms) / 1000.0;
    
    % --- A. 🎯 目标机动逻辑 ---
    if current_time_s >= 1.0
        target_vel = [0.0; 2.5; 0.0]; % 1秒后向右以 2.5m/s 高速逃逸
    else
        target_vel = [0.0; 0.0; 0.0]; 
    end
    target_pos = target_pos + target_vel * dt;
    
    current_yaw = atan2(dart_vel(2), dart_vel(1));
    current_pitch = atan2(-dart_vel(3), sqrt(dart_vel(1)^2 + dart_vel(2)^2));
    fly_q = euler_to_quat(current_yaw, current_pitch, 0); 
    
    is_descending = dart_vel(3) > 0; 
    rel_pos_inertial = target_pos - dart_pos; 
    history_miss_distance(i) = norm(rel_pos_inertial); 
    
    fly_q_conj = [fly_q(1); -fly_q(2); -fly_q(3); -fly_q(4)];
    rel_pos_body = quat_rotate(fly_q_conj, rel_pos_inertial);
    rel_pos_cam = [rel_pos_body(2); rel_pos_body(3); rel_pos_body(1)];
    
    % 像素差（目标像素 - 图像中心），与 C++ 接口语义一致
    pixel_dx = 0.0; pixel_dy = 0.0; is_target_in_fov = false;
    if rel_pos_cam(3) > 0.1 
        angle_x = atan(abs(rel_pos_cam(1)) / rel_pos_cam(3));
        angle_y = atan(abs(rel_pos_cam(2)) / rel_pos_cam(3));
        if angle_x < fov_x_half && angle_y < fov_y_half
            is_target_in_fov = true;
            % 计算归一化坐标 (X/Z, Y/Z)
            norm_x = rel_pos_cam(1) / rel_pos_cam(3);
            norm_y = rel_pos_cam(2) / rel_pos_cam(3);
            % 像素差 = 焦距 * 归一化坐标（已减去图像中心）
            pixel_dx = cam_matrix(1) * norm_x;  % fx * (X/Z)
            pixel_dy = cam_matrix(5) * norm_y;  % fy * (Y/Z)
        end
    end
    
    a_pitch = 0.0; a_yaw = 0.0; 
    
    if is_descending && is_target_in_fov
        if ~has_locked_on
            fprintf('👁️ 目标入场 (t=%.2fs)！启动末端制导！\n', current_time_s);
            has_locked_on = true;
            % 【核心修复】：伪造前一帧时间戳，绕过 C++ 的防呆死锁！
            % 强制让 dt 刚好等于 16ms，确保解算器第一帧就能输出有效指令！
            guide_sys.pre_timestamp = current_time_ms - uint32(dt * 1000); 
            % 重置历史视线角防止首帧角速度爆炸（Yaw & Pitch 都需要）
            guide_sys.prev_los_angle_yaw = atan2(rel_pos_cam(1), rel_pos_cam(3));
            guide_sys.prev_los_angle_pitch = atan2(-rel_pos_cam(2), rel_pos_cam(3));
        end
        history_phase(i) = 2; history_dx_dy(:, i) = [pixel_dx; pixel_dy];
        
        % 固定接近速度（模拟典型弹目接近速度，不受飞镖减速影响）
        FIXED_CLOSING_VEL = 15.0;  % m/s
        guide_sys.guidance_update(pixel_dx, pixel_dy, fly_q(1), fly_q(2), fly_q(3), fly_q(4), FIXED_CLOSING_VEL, current_time_ms);
        out = guide_sys.guidance_calculate();
        
        if isstruct(out) && out.valid
            if ~isempty(out.accel_pitch) && ~isnan(out.accel_pitch), a_pitch = double(out.accel_pitch); end
            if ~isempty(out.accel_yaw) && ~isnan(out.accel_yaw), a_yaw = double(out.accel_yaw); end
        end
        
        % 抓取透视变量
        debug_los_yaw(i) = guide_sys.prev_los_angle_yaw;
        debug_rate_yaw(i) = guide_sys.los_angle_yaw_velocity;
        debug_los_pitch(i) = guide_sys.prev_los_angle_pitch;
        debug_rate_pitch(i) = guide_sys.los_angle_pitch_velocity;
    else
        has_locked_on = false;
        history_phase(i) = is_descending; % 0为爬升, 1为下落盲飞
    end
    
    if ENABLE_GRAVITY_COMPENSATION && history_phase(i) == 2
        g_body = quat_rotate(fly_q_conj, [0; 0; 9.81]);
        a_pitch = a_pitch + g_body(3); 
    end
    
    accel_body = [0; a_yaw; -a_pitch]; 
    accel_inertial = quat_rotate(fly_q, accel_body); 
    total_accel = accel_inertial + [0; 0; 9.81]; 
    
    dart_vel = dart_vel + total_accel * dt;
    dart_pos = dart_pos + dart_vel * dt;
    
    history_dart_pos(:, i) = dart_pos; history_target_pos(:, i) = target_pos; history_accel(:, i) = [a_pitch; a_yaw];
    current_time_ms = current_time_ms + uint32(dt * 1000);
    
    % --- B. 🎯 击中/脱靶判断 ---
    % 同一高度时（Z 坐标接近），判断是否击中 15cm 正方形目标
    z_diff = abs(dart_pos(3) - target_pos(3));
    if z_diff < 0.05  % 高度差 5cm 以内视为同一高度
        dx = abs(dart_pos(1) - target_pos(1));
        dy = abs(dart_pos(2) - target_pos(2));
        horizontal_dist = sqrt(dx^2 + dy^2);
        min_miss_dist = min(min_miss_dist, horizontal_dist);
        
        if dx < TARGET_HALF && dy < TARGET_HALF
            % 击中！
            hit_flag = true; valid_N = i; final_miss_dist = 0;
            fprintf('🎯 击中目标！\n');
            break;
        end
    end
    
    % 飞镖超过目标 X 位置 → 未击中，取最小水平距离作为脱靶量
    if dart_pos(1) >= target_pos(1)
        hit_flag = true; valid_N = i;
        if isinf(min_miss_dist)
            final_miss_dist = history_miss_distance(i);
        else
            final_miss_dist = min_miss_dist;
        end
        fprintf('❌ 未击中！脱靶量: %.3f m (目标半边长 %.3f m)\n', final_miss_dist, TARGET_HALF);
        break;
    end
    
    % 坠地
    if dart_pos(3) > 0.5
        hit_flag = true; valid_N = i;
        if isinf(min_miss_dist)
            final_miss_dist = min(history_miss_distance(1:i));
        else
            final_miss_dist = min_miss_dist;
        end
        fprintf('💥 坠地！脱靶量: %.3f m\n', final_miss_dist);
        break;
    end
end

if ~hit_flag, valid_N = N; end
time_axis = (0:valid_N-1) * dt;

% 输出本次仿真 config 参数
fprintf('📋 仿真参数: N=%.1f, Vc=%.1f m/s, α=%.2f, max_yaw=%.1f, max_pitch=%.1f, 重力补偿=%d\n', ...
    config.nav_ratio, 15.0, config.filter_alpha, ...
    config.max_accel_yaw, config.max_accel_pitch, ENABLE_GRAVITY_COMPENSATION);

%% 4. 数据可视化 (主视图)
figure('Name', '飞镖闭环制导 - 修复版', 'Position', [50, 50, 1100, 700]);
subplot(2,2,1);
idx_ascend = history_phase(1:valid_N) == 0; idx_descend_blind = history_phase(1:valid_N) == 1; idx_guide = history_phase(1:valid_N) == 2;
plot3(history_dart_pos(1, idx_ascend), history_dart_pos(2, idx_ascend), -history_dart_pos(3, idx_ascend), 'k:', 'LineWidth', 1.5); hold on; grid on;
plot3(history_dart_pos(1, idx_descend_blind), history_dart_pos(2, idx_descend_blind), -history_dart_pos(3, idx_descend_blind), 'm--', 'LineWidth', 1.5);
plot3(history_dart_pos(1, idx_guide), history_dart_pos(2, idx_guide), -history_dart_pos(3, idx_guide), 'r', 'LineWidth', 2.5);
plot3(history_target_pos(1, 1:valid_N), history_target_pos(2, 1:valid_N), -history_target_pos(3, 1:valid_N), 'g--', 'LineWidth', 2);
scatter3(history_target_pos(1, valid_N), history_target_pos(2, valid_N), -history_target_pos(3, valid_N), 100, 'g', 'filled');
scatter3(history_dart_pos(1, valid_N), history_dart_pos(2, valid_N), -history_dart_pos(3, valid_N), 80, 'r', 'filled');
title('3D 追逃轨迹'); xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)'); view(20, 25); axis equal; xlim([0 30]);

subplot(2,2,2);
plot(time_axis, history_dx_dy(1, 1:valid_N), 'r.', 'MarkerSize', 8); hold on;
plot(time_axis, history_dx_dy(2, 1:valid_N), 'b.', 'MarkerSize', 8);
xline(1.0, 'k--', '目标机动(1s)', 'LabelVerticalAlignment', 'bottom'); yline(0, 'k:');
grid on; title('相机识别目标像素差 (相对图像中心)'); xlabel('时间 (s)'); ylabel('像素差'); legend('dx (偏航)', 'dy (俯仰)');

subplot(2,2,[3 4]);
stairs(time_axis, history_accel(1, 1:valid_N), 'r', 'LineWidth', 1.5); hold on;
stairs(time_axis, history_accel(2, 1:valid_N), 'b', 'LineWidth', 2.0); % 加粗偏航线
xline(1.0, 'k--'); grid on; title('飞控加速度过载指令 (观察加粗的蓝线)');
xlabel('时间 (s)'); ylabel('指令 (m/s^2)'); legend('a_{pitch}', 'a_{yaw}');

%% 5. 新增：制导律内部算子透视窗 (Yaw & Pitch)
figure('Name', '制导律内部核心参数透视 (Yaw & Pitch)', 'Position', [1200, 50, 800, 900]);
subplot(4,1,1);
plot(time_axis, debug_los_yaw(1:valid_N), 'b', 'LineWidth', 2); grid on;
title('制导律内部: 偏航视线角 (LOS Angle Yaw)'); ylabel('角度 (rad)');
subplot(4,1,2);
plot(time_axis, debug_rate_yaw(1:valid_N), 'r', 'LineWidth', 2); grid on;
title('制导律内部: 偏航视线角速度 (LOS Rate Yaw)'); ylabel('角速度 (rad/s)');
subplot(4,1,3);
plot(time_axis, debug_los_pitch(1:valid_N), 'b', 'LineWidth', 2); grid on;
title('制导律内部: 俯仰视线角 (LOS Angle Pitch)'); ylabel('角度 (rad)');
subplot(4,1,4);
plot(time_axis, debug_rate_pitch(1:valid_N), 'r', 'LineWidth', 2); grid on;
title('制导律内部: 俯仰视线角速度 (LOS Rate Pitch)'); xlabel('时间 (s)'); ylabel('角速度 (rad/s)');

function q = euler_to_quat(yaw, pitch, roll), cy = cos(yaw * 0.5); sy = sin(yaw * 0.5); cp = cos(pitch * 0.5); sp = sin(pitch * 0.5); cr = cos(roll * 0.5); sr = sin(roll * 0.5); q = [cr*cp*cy + sr*sp*sy; sr*cp*cy - cr*sp*sy; cr*sp*cy + sr*cp*sy; cr*cp*sy - sr*sp*cy]; end
function v_rot = quat_rotate(q, v), if isempty(v) || numel(v) < 3, v_rot=[0;0;0]; return; end, q=q(:); v=v(:); qw=q(1); qx=q(2); qy=q(3); qz=q(4); vx=v(1); vy=v(2); vz=v(3); cx1=qy*vz-qz*vy; cy1=qz*vx-qx*vz; cz1=qx*vy-qy*vx; tx=cx1+qw*vx; ty=cy1+qw*vy; tz=cz1+qw*vz; cx2=qy*tz-qz*ty; cy2=qz*tx-qx*tz; cz2=qx*ty-qy*tx; v_rot=[vx+2.0*cx2; vy+2.0*cy2; vz+2.0*cz2]; end
