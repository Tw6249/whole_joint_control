function mujocoKneeClose(plant)
%MUJOCOKNEECLOSE Close the knee plant.

if isa(plant, "MujocoKneePlant")
    plant.close();
end
end
