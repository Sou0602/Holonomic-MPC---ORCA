function position = calc_pos(agent,m,dt)
xvels = agent.xvels;
yvels = agent.yvels;
omegas = agent.omegas;

position = agent.position + [xvels(m) yvels(m) omegas(m)] * dt;



end