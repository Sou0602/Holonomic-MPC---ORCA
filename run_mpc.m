%add agent with initial pose , goal and vmax.
agent = addAgent('1',[3,4,pi/6],[0,0,0],[7,-9,pi/4],1);

%N - Number of MPC predictions to make
agent.N = 30;

%M - Frequency of replanning
agent.M = 10;

%Time Step 
dt = 0.1;
counter = 0;
m = 1;
maxIterations = 500 * agent.M;
axisLimits = [-10 10 -10 10];
Vel = [];
Theta = [];
while counter < maxIterations
%MPC Planner - Holonomic 
agent.velocities = comp_mpc(agent,dt);
[agent.xvels , agent.yvels, agent.omegas] = getnvels(agent);

for i = 1:agent.M
agent.path = [agent.path ; agent.position];
agent.position = calc_pos(agent,m,dt);
plotSimulation(agent,counter,dt,axisLimits,false,m);
Vel = [Vel;agent.xvels(m),agent.yvels(m)];
Theta = [Theta ; agent.position(3)];
if m < agent.M
m = m + 1;
else
    m = 1;
end
distfromgoal = sqrt( sum((agent.position - agent.goal).^2));
counter = counter + 1;
end

if distfromgoal < 0.2
   break
end

end


