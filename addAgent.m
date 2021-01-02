function agent = addAgent(name, position, velocity, goal,vmax)
% addAgent - Return a struct with agent details
%
% Syntax: agent = addAgent(name, position, velocity, goal)
%
    agent = struct( 'name', name, 'position', position, 'velocity', velocity );
    agent.goal = goal;
    agent.path = [];
    agent.radius = 0.5;
    agent.sensorRange = 10;
    agent.vmax = 1.5;
    agent.wmax = 1;
    agent.N = 50;
    agent.M = 10;
    agent.initialpos = agent.position;
    agent.xvels = [];
    agent.yvels = [];
    agent.omegas = [];
    agent.velocities = zeros(1,3*agent.N);
    agent.w = 0;
    
    
    agent.phi=atan2((agent.goal(2)-agent.position(2)),(agent.goal(1)-agent.position(1)));
    agent.color=[0 170 255] / 255;
    agent.prevfov={ };
    agent.sen_angle=45;
    agent.time=0;
    agent.count=0;
    agent.obstacle_indx=[];
    agent.coeff=[];
    agent.initialpos = position;
    agent.rpcounter = 1;
    agent.refpath = [];
    agent.gflag = 0;
    agent.goaldist = 100;
    agent.vel = [0,0];
    agent.obs = [];
end