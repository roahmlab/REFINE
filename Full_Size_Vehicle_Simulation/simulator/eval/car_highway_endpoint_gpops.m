function output = car_highway_endpoint_gpops(input)

q  = input.phase.finalstate(:);
waypt = input.parameter(:);

output.objective = sum((q(1:2) - waypt(1:2)).^2);