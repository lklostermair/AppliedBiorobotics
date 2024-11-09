classdef SingleLeg < matlab.System

    properties
        left; % Indicates if this is the left leg (boolean)
        alpha_TD; % Angle of touchdown (in radians)
        k;  % Spring constant
    end

    properties (Access = private)
        x_FP = -1; % Foot Point x-coordinate (in meters)
        stance_phase = false; % Indicates if the leg is in the stance phase
        l0 = 1; % Rest length of the spring (in meters)
    end

    methods (Access = protected)
        function [Fs_x, Fs_y, stance_phase, delta_l] = stepImpl(obj, x, y, dy, t)
            % Calculate the forces and update the stance phase
            delta_l = 0;
            if obj.stance_phase
                l = sqrt((x - obj.x_FP)^2 + y^2);
                delta_l = obj.l0 - l;
                Fs = obj.k * delta_l;
                Fs_x = Fs * (x - obj.x_FP) / l;
                Fs_y = Fs * y / l;
                if l >= obj.l0 && dy > 0
                    obj.stance_phase = false;
                end
            else
                Fs_x = 0;
                Fs_y = 0;
                if (y - obj.l0 * sin(obj.alpha_TD)) <= 0 && dy < 0
                    obj.x_FP = x + obj.l0 * cos(obj.alpha_TD);
                    obj.stance_phase = true;
                end
            end
            stance_phase = obj.stance_phase;
        end

        function resetImpl(obj)
            % Initialize or reset internal properties
            if obj.left
                obj.x_FP = 0;
                obj.stance_phase = true;
            else
                obj.stance_phase = false;
            end
        end
    end

    methods
        function [Fs_x, Fs_y, stance_phase, delta_l] = computeForces(obj, x, y, dy, t)
            % Public method to calculate forces
            [Fs_x, Fs_y, stance_phase, delta_l] = obj.stepImpl(x, y, dy, t);
        end

        function resetLeg(obj)
            % Public method to reset the leg
            obj.resetImpl();
        end
    end
end
