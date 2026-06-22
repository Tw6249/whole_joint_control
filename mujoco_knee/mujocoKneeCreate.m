function plant = mujocoKneeCreate(xmlPath, Ts)
%MUJOCOKNEECREATE Create the knee plant used by eid_control.m.
%
% This MATLAB implementation is a deterministic fallback for the original
% MuJoCo/MEX plant API. It uses the same fitted single-joint dynamics as the
% observer in eid_control.m, so the script can run without a compiled MEX.

plant = MujocoKneePlant(xmlPath, Ts);
end
