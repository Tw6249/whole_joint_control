function [plant, q, dq] = mujocoKneeReset(plant, q0, dq0)
%MUJOCOKNEERESET Reset the knee plant state.

validatePlant(plant);
[q, dq] = plant.reset(q0, dq0);
end

function validatePlant(plant)
if ~isa(plant, "MujocoKneePlant")
    error("mujoco_knee:InvalidPlant", "plant must be a MujocoKneePlant instance.");
end
end
