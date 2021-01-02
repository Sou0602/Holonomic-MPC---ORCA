function costs = getcosts(u,agent,dt)
costs = [];

N = agent.N;

xvels = u(1:N);
yvels = u(N+1:2*N);
omegas = u(2*N+1:3*N);

[Xn, Yn , Thetan] = Calc_Npos(agent,u,dt);
Xg = agent.goal(1);
Yg = agent.goal(2);
Thetag = agent.goal(3);

costs = [(Xn - Xg)^2+(Yn - Yg)^2 , (Thetan - Thetag)^2];
costs = [costs , velcost(u,agent) , omegacost(u,agent)];
costs = [costs, sum(xvels.^2) , sum(yvels.^2), sum(omegas.^2)];

end