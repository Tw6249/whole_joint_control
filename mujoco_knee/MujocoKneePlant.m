classdef MujocoKneePlant < handle
    properties
        xmlPath string
        Ts double = 0.002
        q double = 0.9
        dq double = 0.0
        J double = 0.238
        b double = 1.0
        gravityA double = 4.2835
        gravityB double = 0.0
        tau0 double = -0.2711
        tauLimit double = 300.0
        qMin double = -0.26
        qMax double = 2.05
        useTorqueLimit logical = true
        usePositionLimit logical = true
        isClosed logical = false
    end

    methods
        function obj = MujocoKneePlant(xmlPath, Ts)
            if nargin >= 1 && strlength(string(xmlPath)) > 0
                obj.xmlPath = string(xmlPath);
            else
                obj.xmlPath = "";
            end
            if nargin >= 2 && ~isempty(Ts)
                obj.Ts = double(Ts);
            end
        end

        function [q, dq] = reset(obj, q0, dq0)
            obj.assertOpen();
            obj.q = double(q0);
            obj.dq = double(dq0);
            [obj.q, obj.dq] = obj.enforcePositionLimit(obj.q, obj.dq);
            q = obj.q;
            dq = obj.dq;
        end

        function out = step(obj, tauRaw)
            obj.assertOpen();
            tauApplied = double(tauRaw);
            if obj.useTorqueLimit
                tauApplied = min(max(tauApplied, -obj.tauLimit), obj.tauLimit);
            end

            qacc = (tauApplied ...
                - obj.b * obj.dq ...
                - obj.gravityA * sin(obj.q) ...
                - obj.gravityB * cos(obj.q) ...
                - obj.tau0) / obj.J;
            dqNext = obj.dq + obj.Ts * qacc;
            qNext = obj.q + obj.Ts * dqNext;
            if obj.usePositionLimit
                [qNext, dqNext] = obj.enforcePositionLimit(qNext, dqNext);
            end

            obj.q = qNext;
            obj.dq = dqNext;
            out = struct( ...
                "q_next", qNext, ...
                "dq_next", dqNext, ...
                "qacc", qacc, ...
                "tau_applied", tauApplied);
        end

        function close(obj)
            obj.isClosed = true;
        end
    end

    methods (Access = private)
        function assertOpen(obj)
            if obj.isClosed
                error("mujoco_knee:PlantClosed", "mujoco knee plant has already been closed.");
            end
        end

        function [q, dq] = enforcePositionLimit(obj, q, dq)
            if q < obj.qMin
                q = obj.qMin;
                if dq < 0.0
                    dq = 0.0;
                end
            elseif q > obj.qMax
                q = obj.qMax;
                if dq > 0.0
                    dq = 0.0;
                end
            end
        end
    end
end
