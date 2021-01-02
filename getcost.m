function cost = getcost(agent,u,dt)
[Xn, Yn , Thetan] = Calc_Npos(agent,u,dt);
Xg = agent.goal(1);
Yg = agent.goal(2);
Thetag = agent.goal(3);

cost = 1/65*((Xn - Xg)^2 + (Yn - Yg)^2 )+ 10*(Thetan - Thetag)^2;
end