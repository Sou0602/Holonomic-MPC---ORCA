function [c,ceq] = euclidean(agent,u,dt)

c = [];
ceq = [];
N = agent.N;
for i = 1:length(agent.obs)
    
    obstacle = agent.obs(i);
    obs_vel = obstacle.vel;
    obs_pos = obstacle.position(1:2);
    agent_pos = agent.position(1:2);

 for j = 1:N
        vx = u(j);
        vy = u(N+j);
        
        agent_vel = [vx , vy];
        agent_pos = agent_pos + agent_vel * dt;
        obs_pos = obs_pos + obs_vel * dt;
        dist = sqrt(sum(agent_pos - obs_pos).^2);
        
        
           c(end+1)=-1*(dist - (agent.radius+obstacle.radius));
 
 end
        
        
end

end
        