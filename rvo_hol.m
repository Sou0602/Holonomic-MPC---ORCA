function [c,ceq] = rvo_hol(agent,u,dt)

c = [];
ceq = [];
N = agent.N;
for i = 1:length(agent.obs)
    
    obstacle = agent.obs(i);
    obs_vel = obstacle.vel;
    obs_pos = obstacle.position(1:2);
    agent_pos = agent.position(1:2);
    agent_v0 = agent.vel;
 for j = 1:N
        vrvo = [u(j) u(N+j)]; 
        if j > 1
        obs_pos = obs_pos + obs_vel * dt;
        agent_v0 = [u(j-1) u(N+j-1)];
        agent_pos = agent_pos + agent_v0 * dt;
        end
        
        rij = (agent_pos - obs_pos) ; 
        vdiff = (2*vrvo - agent_v0 - obs_vel);
        
        rij2 = sum(rij.^2);
        vdiff2 = sum(vdiff.^2);
        Rij2 = agent.radius + obstacle.radius;
        f = ( vdiff2 * (rij2 - Rij2) ) - ( vdiff * rij' )^2;
        
           c(end+1)=-1*(( vdiff2 * (rij2 - Rij2) ) - ( vdiff * rij' )^2 );
 
 end         
end
end