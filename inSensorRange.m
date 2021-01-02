function out = inSensorRange(agent, obstacle)
% inSensorRange - Returns true if the obstalce is in sensor range
%
% Syntax: out = inSensorRange(agent, obstacle)
%
    distance = sum((agent.position(1:2) - obstacle.position(1:2)).^2) < agent.sensorRange^2;
    sameSide = (agent.position(1:2) - obstacle.position(1:2)) * (agent.vel - obstacle.vel)' < 0;
    out = distance && sameSide;
    
    %
    if sqrt(sum((agent.position(1:2) - obstacle.position(1:2)).^2)) <= 4*agent.radius 
        out =1;
    end
    %}
end