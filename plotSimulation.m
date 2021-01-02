function plotSimulation(agents, counter, dt, axisLimits, save,m)
% plotSimulation - Plots the positions of the agents and their paths
%
% Syntax: plotSimulation(agents, counter, dt, axisLimits, save)
%
    clf('reset')
    hold on
%    agent_grid=zeros(length(agents));
    for i = 1:length(agents)
        faceColor = [0 170 255] / 255;
        lineColor = [135 135 135] / 255;
        filledCircle(agents(i).position, agents(i).radius, 1000, faceColor);
       % plot(linspace(agents(i).path(1,1),agents(i).path(end,1),50),linspace(agents(i).path(1,2),agents(i).path(end,2),50))
        vx = agents(i).xvels(m);
        vy = agents(i).yvels(m);
        quiver(agents(i).position(1), agents(i).position(2),vx ,vy , 'Color', [0 0 0]);
        plot(agents(i).path(:, 1), agents(i).path(:, 2), 'Color', lineColor);
      
 
       Xg = [agents(i).initialpos(1) ; agents(i).goal(1)]; 
       Yg = [agents(i).initialpos(2) ; agents(i).goal(2)]; 
        plot(Xg,Yg,'b--','LineWidth',0.1);
        
%         for j = 1:length(agents)
%             if (i~=j)%(i==1&&j==4) || (i==4&&j==1)
%                 vRel = agents(i).velocity - agents(j).velocity;
%                 
%                 pAb = (agents(j).position - agents(i).position) ;
%                        pAblen=sqrt(sum(pAb.^2));
%                 if (inSensorRange(agents(i), agents(j)) && ((pAblen^2-((vRel(1)*pAb(1)+vRel(2)*pAb(2))^2)/sum(vRel.^2))<=2^2))
%                  agent_grid(i,j)=1;
%                  agent_grid(j,i)=1;
%                  tangent(agents(i).position, agents(j).position, agents(i).radius+agents(j).radius, agents(j).radius,agents(i).sensorRange);
%                  
%                 end
%             end
%         end
        %if(~isempty(agents(i).prevfov))
        %    plot(agents(i).prevfov, 'FaceColor', [0.86,0.5,0.1])
        %end
        text(agents(i).position(1), agents(i).position(2), agents(i).name);
    end
    title([ 'Time: ' num2str((counter*agents(1).M + m)*dt) 's' ])
    set(get(gca, 'XLabel'), 'String', 'X [m]');
    set(get(gca, 'YLabel'), 'String', 'Y [m]');
    axis(axisLimits)
    axis equal
    hold off
    drawnow
%
     if save
         saveas(gcf, ['4agents_rvo-2/', num2str(counter*agents(1).M + m,'%04.f'), '.png']);
     end
    %}
end