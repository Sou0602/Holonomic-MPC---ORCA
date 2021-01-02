function vels = comp_mpc(agent,dt)
N = agent.N;
cost = @(u) 50* 1/2*getcost(agent,u',dt) + 0.5*1/2.2 * velcost(u',agent) +0.5*1/1.2 *omegacost(u',agent) + 1/12*sum(u(1:N).^2) + 1/12*sum(u(N+1:2*N).^2) + 1/100*sum(u(2*N +1 : 3*N).^2);

init = agent.velocities' ;
lb = ones(1,2*N) * -agent.vmax;
lb = [lb , ones(1,N) * -agent.wmax];
ub = ones(1,3*N) * (agent.vmax);

% xvels = agent.velocities(1:N)
% yvels = agent.velocities(N+1:2N)
A = [];
B = [];
Aeq = [];
Beq = [];

constraints = [];

%vx acceleration

for i = 1 : N - 1
    ax = zeros(1,3*N);
    ax(i+1) = 1;
    ax(i) = -1;
    bx = dt * 1;
    A = [A;ax];
    B = [B;bx];
end
%deceleration
for i = 1 : N -1
    ax = zeros(1,3*N);
    ax(i+1) = -1;
    ax(i) = 1;
    bx = dt * 1.5;
    A = [A;ax];
    B = [B;bx];
end
%vy acceleration
for i = 1 : N -1
    ax = zeros(1,3*N);
    ax(N+i+1) = 1;
    ax(N+i) = -1;
    bx = dt * 1;
    A = [A;ax];
    B = [B;bx];
end
%deceleration
for i = 1 : N -1
    ax = zeros(1,3*N);
    ax(N+i+1) = -1;
    ax(N+i) = 1;
    bx = dt * 1.5;
    A = [A;ax];
    B = [B;bx];
end
%angular acceleration 
for i = 1 : N -1
    ax = zeros(1,3*N);
    ax(2*N+i+1) = 1;
    ax(2*N+i) = -1;
    bx = dt * 1/5;
    A = [A;ax];
    B = [B;bx];
end
%deceleration
for i = 1 : N -1
    ax = zeros(1,3*N);
    ax(2*N+i+1) = -1;
    ax(2*N+i) = 1;
    bx = dt * 1/5;
    A = [A;ax];
    B = [B;bx];
end

%{
%vx,vy,w continuity
vx = agent.vel(1);
vy = agent.vel(2);
w = agent.w;
aeq1 = zeros(1,3*N);
aeq1(1) = 1;
beq1 = vx;
Aeq = [Aeq;aeq1];
Beq = [Beq;beq1];
aeq2 = zeros(1,3*N);
aeq2(1 + N) = 1;
beq2 = vy;
Aeq = [Aeq;aeq2];
Beq = [Beq;beq2];
aeq3 = zeros(1,3*N);
aeq3(1 + 2*N) = 1;
beq3 = w;
Aeq = [Aeq;aeq3];
Beq = [Beq;beq3];
%linear approx
%}
%Linear constraints
%
if length(agent.obs) ~= 0
    
for i = 1:length(agent.obs)
    obstacle = agent.obs(i);
    obs_vel = obstacle.vel;
    obs_pos = obstacle.position(1:2);
    agent_pos = agent.position(1:2);
    r0 = norm(agent_pos - obs_pos);
  %  agent_vel0 = agent.vel;
    for j = 1:length(N)
      agent_vel0 = [agent.velocities(j) , agent.velocities(N+j)];
      v = [];
      nor = [];
      [v,nor] =   getorca_lin(agent,agent_pos,obs_pos,agent_vel0,obs_vel,j,N);
      a = zeros(1,3*agent.N);     
      a(j) = -nor(1);
      a(N+j) = -nor(2);
      b = -(agent_vel0(1)+v(1)*0.5)*nor(1) -(agent_vel0(2)+v(2)*0.5)*nor(2) ;
      A = [A;a];
      B = [B;b];
      agent_pos = agent_pos + agent_vel0 * dt;
      obs_pos = obs_pos + obs_vel*dt;

    end
end


end
%}

%non-linear constraints
if length(agent.obs) ~= 0

%constraints = @(u) getorca_nonlin(agent,N,u,dt);
%constraints = @(u) euclidean(agent,u,dt);
%constraints =  @(u) rvo_hol(agent,u,dt); 
end
%}


    options = optimoptions('fmincon','Display','final-detailed','Algorithm','interior-point');
 
    vels = fmincon(cost, init, A, B, Aeq, Beq, lb, ub, constraints, options)';
   % vels = fmincon(cost, init, A, B, [], [], lb, ub, [], options)';

    vels = smoothvels(vels,agent);
end
