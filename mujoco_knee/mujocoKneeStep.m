function out = mujocoKneeStep(plant, tauRaw)
%MUJOCOKNEESTEP Advance the knee plant by one control sample.

validatePlant(plant);
out = plant.step(tauRaw);
end

function validatePlant(plant)
if ~isa(plant, "MujocoKneePlant")
    error("mujoco_knee:InvalidPlant", "plant must be a MujocoKneePlant instance.");
end
end
