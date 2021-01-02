function cost = omegacost(u,agent)
costth = 0;
N = length(u)/3;
for i = 2:N
    costth = costth + (u(i+2*N) - u(2*N+i-1))^2;
end
costth = costth + (u(2*N + 1) - agent.w)^2 ; 
 cost = costth;
end