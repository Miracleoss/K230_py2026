classdef Guidance < handle
    properties
        g_config, g_input, g_output
        pre_timestamp, prev_los_angle_yaw, prev_los_angle_pitch
        % 新增：用于正确滤波的历史角速度（主脚本透视图需要读取它）
        los_angle_yaw_velocity, los_angle_pitch_velocity
        reference_quat, is_fire, initialized, reference_set
    end
    
    methods
        function obj = Guidance()
            obj.initialized = false; obj.reference_set = false; obj.is_fire = false;
            obj.pre_timestamp = 0; obj.prev_los_angle_yaw = 0.0; obj.prev_los_angle_pitch = 0.0;
            obj.los_angle_yaw_velocity = 0.0; obj.los_angle_pitch_velocity = 0.0;
        end
        
        function guidance_init(obj, config, ref_qw, ref_qx, ref_qy, ref_qz)
            obj.g_config = config;
            obj.reference_quat = obj.quaternion_normalize([ref_qw, ref_qx, ref_qy, ref_qz]);
            obj.initialized = true; obj.reference_set = true; obj.is_fire = false;
            obj.pre_timestamp = 0; obj.prev_los_angle_yaw = 0.0; obj.prev_los_angle_pitch = 0.0;
            obj.los_angle_yaw_velocity = 0.0; obj.los_angle_pitch_velocity = 0.0;
            obj.g_output.accel_pitch = 0.0; obj.g_output.accel_yaw = 0.0; obj.g_output.valid = false;
            obj.g_input.pixel_dx = 0.0; obj.g_input.pixel_dy = 0.0;
            obj.g_input.q = [1.0, 0.0, 0.0, 0.0]; obj.g_input.target_velocity = 0.0; obj.g_input.timestamp = 0;
        end
        
        function fire(obj), obj.is_fire = true; end
        
        function guidance_update(obj, dx, dy, qw, qx, qy, qz, target_velocity, timestamp)
            obj.g_input.pixel_dx = dx; obj.g_input.pixel_dy = dy; obj.g_input.timestamp = timestamp;
            obj.g_input.q = obj.quaternion_normalize([qw, qx, qy, qz]);
            obj.g_input.target_velocity = target_velocity;
        end
        
        function output = guidance_calculate(obj)
            if ~obj.is_fire || ~obj.initialized || ~obj.reference_set || obj.g_input.timestamp == 0
                output.accel_pitch = 0; output.accel_yaw = 0; output.valid = false; return;
            end
            
            if obj.pre_timestamp == 0
                obj.pre_timestamp = obj.g_input.timestamp;
                output.accel_pitch = 0; output.accel_yaw = 0; output.valid = true; return;
            end
            
            dt = double(obj.g_input.timestamp - obj.pre_timestamp) / 1000.0;
            if dt <= 0.0 || dt > 1.0
                output.accel_pitch = 0; output.accel_yaw = 0; output.valid = false; return;
            end
            
            cam_x = obj.g_input.pixel_dx / obj.g_config.camera_matrix(1);
            cam_y = obj.g_input.pixel_dy / obj.g_config.camera_matrix(5);
            cam_z = 1.0;
            
            dart_x = cam_z; dart_y = cam_x; dart_z = cam_y;
            
            rel_q = obj.quaternion_relative(obj.g_input.q, obj.reference_quat);
            qw = rel_q(1); qx = rel_q(2); qy = rel_q(3); qz = rel_q(4);
            
            C00 = 1.0 - 2.0 * (qy^2 + qz^2); C01 = 2.0 * (qx*qy - qw*qz); C02 = 2.0 * (qx*qz + qw*qy);
            C10 = 2.0 * (qx*qy + qw*qz); C11 = 1.0 - 2.0 * (qx^2 + qz^2); C12 = 2.0 * (qy*qz - qw*qx);
            C20 = 2.0 * (qx*qz - qw*qy); C21 = 2.0 * (qy*qz + qw*qx); C22 = 1.0 - 2.0 * (qx^2 + qy^2);
            
            inertial_x = C00 * dart_x + C01 * dart_y + C02 * dart_z;
            inertial_y = C10 * dart_x + C11 * dart_y + C12 * dart_z;
            inertial_z = C20 * dart_x + C21 * dart_y + C22 * dart_z;
            
            los_angle_yaw = obj.safe_atan2(inertial_y, inertial_x);
            los_angle_pitch = obj.safe_atan2(-inertial_z, sqrt(inertial_x^2 + inertial_y^2));
            
            % 计算瞬时角速度
            raw_los_rate_yaw = (los_angle_yaw - obj.prev_los_angle_yaw) / dt;
            raw_los_rate_pitch = (los_angle_pitch - obj.prev_los_angle_pitch) / dt;
            
            % 使用正确的历史角速度进行滤波
            obj.los_angle_yaw_velocity = obj.low_pass_filter(raw_los_rate_yaw, obj.los_angle_yaw_velocity, obj.g_config.filter_alpha);
            obj.los_angle_pitch_velocity = obj.low_pass_filter(raw_los_rate_pitch, obj.los_angle_pitch_velocity, obj.g_config.filter_alpha);
            
            n_yaw = obj.g_config.nav_ratio * obj.g_input.target_velocity * obj.los_angle_yaw_velocity;
            n_pitch = obj.g_config.nav_ratio * obj.g_input.target_velocity * obj.los_angle_pitch_velocity;
            
            n_yaw = obj.clamp_float(n_yaw, -obj.g_config.max_accel_yaw, obj.g_config.max_accel_yaw);
            n_pitch = obj.clamp_float(n_pitch, -obj.g_config.max_accel_pitch, obj.g_config.max_accel_pitch);
            
            obj.prev_los_angle_yaw = los_angle_yaw;
            obj.prev_los_angle_pitch = los_angle_pitch;
            obj.pre_timestamp = obj.g_input.timestamp;
            
            output.accel_pitch = n_pitch; output.accel_yaw = n_yaw; output.valid = true;
            obj.g_output = output;
        end
        
        function guidance_reset(obj)
            obj.initialized = false; obj.reference_set = false; obj.is_fire = false;
            obj.pre_timestamp = 0; obj.prev_los_angle_yaw = 0.0; obj.prev_los_angle_pitch = 0.0;
            obj.los_angle_yaw_velocity = 0.0; obj.los_angle_pitch_velocity = 0.0;
            obj.g_output.valid = false;
        end
    end
    
    methods (Access = private)
        function val = clamp_float(~, val, min_v, max_v), val = max(min(val, max_v), min_v); end
        function a = safe_atan2(~, y, x), if abs(x)<eps && abs(y)<eps, a=0.0; else, a=atan2(y, x); end, end
        function nq = quaternion_normalize(~, q), nval = norm(q); if nval>eps, nq=q/nval; else, nq=[1,0,0,0]; end, end
        function cq = quaternion_conjugate(~, q), cq = [q(1), -q(2), -q(3), -q(4)]; end
        function res = quaternion_multiply(~, a, b)
            res = zeros(1,4); res(1) = a(1)*b(1)-a(2)*b(2)-a(3)*b(3)-a(4)*b(4); res(2) = a(1)*b(2)+a(2)*b(1)+a(3)*b(4)-a(4)*b(3); res(3) = a(1)*b(3)-a(2)*b(4)+a(3)*b(1)+a(4)*b(2); res(4) = a(1)*b(4)+a(2)*b(3)-a(3)*b(2)+a(4)*b(1);
        end
        function rel_q = quaternion_relative(obj, curr, ref), ref_conj = obj.quaternion_conjugate(ref); rel_q = obj.quaternion_multiply(ref_conj, curr); end
        function f = low_pass_filter(obj, in, prev, alpha), alpha = obj.clamp_float(alpha, 0.0, 1.0); f = prev + alpha * (in - prev); end
    end
end
