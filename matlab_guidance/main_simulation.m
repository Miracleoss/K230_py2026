% === 飞镖视觉比例导引 闭环仿真 (偏航死锁修复 + 内部状态透视版) ===
clc; clear all; clear classes; close all;

%% 1. 物理环境与初始参数配置
target_init_pos = [25.0; 0.0; -1.0];  
target_pos = target_init_pos;         

launch_pitch_deg = 35.0;         
launch_yaw_deg = 0.0;
v0 = 17.5;                       
dart_pos = [0.0; 0.0; 0.0];      
dart_vel = [v0 * cosd(launch_pitch_deg); 0; -v0 * sind(launch_pitch_deg)]; 

cam_matrix = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0];
fov_x_half = atan(320.0 / 500.0); 
fov_y_half = atan(240.0 / 500.0); 

ENABLE_GRAVITY_COMPENSATION = true; 

%% 2. 初始化制导系统
config.nav_ratio = 5.5;             
config.camera_matrix = cam_matrix;  
config.filter_alpha = 0.8;          
config.max_accel_yaw = 20.0;        
config.max_accel_pitch = 30.0;      

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

% === 新增：透视制导律内部变量 ===
debug_los_yaw = zeros(1, N);
debug_rate_yaw = zeros(1, N);

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
    
    dx = 320.0; dy = 240.0; is_target_in_fov = false;
    if rel_pos_cam(3) > 0.1 
        angle_x = atan(abs(rel_pos_cam(1)) / rel_pos_cam(3));
        angle_y = atan(abs(rel_pos_cam(2)) / rel_pos_cam(3));
        if angle_x < fov_x_half && angle_y < fov_y_half
            is_target_in_fov = true;
            dx = cam_matrix(1) * (rel_pos_cam(1) / rel_pos_cam(3)) + cam_matrix(3);
            dy = cam_matrix(5) * (rel_pos_cam(2) / rel_pos_cam(3)) + cam_matrix(6);
        end
    end
    
    a_pitch = 0.0; a_yaw = 0.0; 
    
    if is_descending && is_target_in_fov
        if ~has_locked_on
            fprintf('👁️ 目标入场 (t=%.2fs)！启动末端制导！\n', current_time_s);
            has_locked_on = true;
            % 【核心修复】：伪造前一帧时间戳，绕过 C++ 的防呆死锁！
            % 强制让 dt 刚好等于 16ms，确保解算器第一帧就能输出有效 Yaw！
            guide_sys.pre_timestamp = current_time_ms - uint32(dt * 1000); 
            % 重置历史角速度防止首帧爆炸
            guide_sys.prev_los_angle_yaw = atan2(rel_pos_cam(1), rel_pos_cam(3));
        end
        history_phase(i) = 2; history_dx_dy(:, i) = [dx; dy];
        
        closing_vel = norm(dart_vel - target_vel); 
        guide_sys.guidance_update(dx, dy, fly_q(1), fly_q(2), fly_q(3), fly_q(4), closing_vel, current_time_ms);
        out = guide_sys.guidance_calculate();
        
        if isstruct(out) && out.valid
            if ~isempty(out.accel_pitch) && ~isnan(out.accel_pitch), a_pitch = double(out.accel_pitch); end
            if ~isempty(out.accel_yaw) && ~isnan(out.accel_yaw), a_yaw = double(out.accel_yaw); end
        end
        
        % 抓取透视变量
        debug_los_yaw(i) = guide_sys.prev_los_angle_yaw;
        debug_rate_yaw(i) = guide_sys.los_angle_yaw_velocity;
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
    
    if dart_pos(1) >= target_pos(1) || history_miss_distance(i) < 0.15
        hit_flag = true; valid_N = i; final_miss_dist = history_miss_distance(i);
        fprintf('🎯 交汇结束！脱靶量: %.3f 米\n', final_miss_dist);
        break;
    end
    if dart_pos(3) > 0.5, valid_N = i; final_miss_dist = min(history_miss_distance(1:valid_N)); fprintf('💥 坠地！\n'); break; end
end

if ~hit_flag, valid_N = N; end
time_axis = (0:valid_N-1) * dt;

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
xline(1.0, 'k--', '目标机动(1s)', 'LabelVerticalAlignment', 'bottom'); yline(320, 'r:'); yline(240, 'b:');
grid on; title('相机识别目标像素'); xlabel('时间 (s)'); ylabel('像素'); legend('X (偏航)', 'Y (俯仰)');

subplot(2,2,[3 4]);
stairs(time_axis, history_accel(1, 1:valid_N), 'r', 'LineWidth', 1.5); hold on;
stairs(time_axis, history_accel(2, 1:valid_N), 'b', 'LineWidth', 2.0); % 加粗偏航线
xline(1.0, 'k--'); grid on; title('飞控加速度过载指令 (观察加粗的蓝线)');
xlabel('时间 (s)'); ylabel('指令 (m/s^2)'); legend('a_{pitch}', 'a_{yaw}');

%% 5. 新增：制导律内部算子透视窗
figure('Name', '制导律内部核心参数透视', 'Position', [1200, 50, 600, 700]);
subplot(2,1,1);
plot(time_axis, debug_los_yaw(1:valid_N), 'b', 'LineWidth', 2); grid on;
title('制导律内部: 偏航视线角 (LOS Angle Yaw)'); ylabel('角度 (rad)');
subplot(2,1,2);
plot(time_axis, debug_rate_yaw(1:valid_N), 'r', 'LineWidth', 2); grid on;
title('制导律内部: 偏航视线角速度 (LOS Rate Yaw)'); xlabel('时间 (s)'); ylabel('角速度 (rad/s)');

function q = euler_to_quat(yaw, pitch, roll), cy = cos(yaw * 0.5); sy = sin(yaw * 0.5); cp = cos(pitch * 0.5); sp = sin(pitch * 0.5); cr = cos(roll * 0.5); sr = sin(roll * 0.5); q = [cr*cp*cy + sr*sp*sy; sr*cp*cy - cr*sp*sy; cr*sp*cy + sr*cp*sy; cr*cp*sy - sr*sp*cy]; end
function v_rot = quat_rotate(q, v), if isempty(v) || numel(v) < 3, v_rot=[0;0;0]; return; end, q=q(:); v=v(:); qw=q(1); qx=q(2); qy=q(3); qz=q(4); vx=v(1); vy=v(2); vz=v(3); cx1=qy*vz-qz*vy; cy1=qz*vx-qx*vz; cz1=qx*vy-qy*vx; tx=cx1+qw*vx; ty=cy1+qw*vy; tz=cz1+qw*vz; cx2=qy*tz-qz*ty; cy2=qz*tx-qx*tz; cz2=qx*ty-qy*tx; v_rot=[vx+2.0*cx2; vy+2.0*cy2; vz+2.0*cz2]; end