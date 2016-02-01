
distance = 0:0.01:1; % from 0 to 1m
f50 = figure(50); clf(f50); set(f50,'Name','Collision risk function');
hold on;
for i=1: length(distance)
    plot(distance(i), collisionRiskFunction(distance(i)));
end
hold off;