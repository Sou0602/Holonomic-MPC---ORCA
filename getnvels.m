function [xvels, yvels , omegas] = getnvels(agent)
velocities = agent.velocities;
M = agent.M;
N = agent.N;
xvels = [];
yvels = [];
omegas = [];

for i = 1:N
xvels = [xvels ; velocities(i)];
yvels = [yvels ; velocities(N + i)];
omegas = [omegas ; velocities(2*N + i)];
end

end