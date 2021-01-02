function [Xn,Yn,Thetan] = Calc_Npos(agent,u,dt)
xvelocities = [];
yvelocities = [];
omegas = [];


for i = 1 : length(agent.N)
    xvelocities = [xvelocities;u(i)];
    yvelocities = [yvelocities;u(agent.N + i)];
    omegas = [omegas;u(2*agent.N + i)];
end

%Position calculation
x = agent.position(1);
y = agent.position(2);
theta = agent.position(3);

for j = 1 : length(xvelocities)
    x = x + xvelocities(j)*dt;
    y = y + yvelocities(j)*dt;
    theta = theta + omegas(j)*dt;
end

Xn = x;
Yn = y;
Thetan = theta;

end
