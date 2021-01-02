function [c,ceq] = getorca_nonlin(agent,N,u,dt)

c = [];
ceq = [];

for i = 1:length(agent.obs)
    
    obstacle = agent.obs(i);
    obs_vel = obstacle.vel;
    obs_pos = obstacle.position(1:2);
    agent_pos = agent.position(1:2);
    agent_vel0 = agent.vel;

 for j = 1:N
     
        agent_vel = [u(j) u(N+j)]; 
        if j > 1
        obs_pos = obs_pos + obs_vel * dt;
        agent_vel0 = [u(j-1) u(N+j-1)];
          for k = 2:j
           agent_pos = agent_pos + [u(k-1) u(N+k-1)] * dt;
          end
        end
        
        tau = 0.8*2 ;
        % Refer the paper for explanation on these terms
        vRel = agent_vel0 - obs_vel;
        pAb = ( -agent_pos + obs_pos)/tau;
        
        % Finding pAb perpendicular
        r = 4*agent.radius/ tau + 0.2/tau ;
        l = abs(sqrt(sum(pAb.^2) - r^2));

        pAblen = sqrt(sum(pAb.^2));
        m = [
            l -r;
            r  l
        ];

        qL = (pAb * m') * ( 1/ sum(pAb.^2));
        qR = (pAb * m ) * ( 1/ sum(pAb.^2));
        pAbL = [qL(2) -qL(1)];
        pAbR = [qR(2) -qR(1)];
     
        %collision cone check
        if ((pAblen)^2-(dot(pAb,vRel)^2)/sum(vRel.^2))<=r^2  

        
            d1=abs((qL(2)*vRel(1) - qL(1)*vRel(2))/sqrt(qL(1)^2+qL(2)^2));
            d2=abs((qR(2)*vRel(1) - qR(1)*vRel(2))/sqrt(qR(1)^2+qR(2)^2));
            d3=Inf;
            w=vRel-pAb;
        %Selecting the direction of minimum distance to the boundary of the
        %collision cone.
            if(pAblen>r)
                if dot(w,pAb)<0 && norm(w)<r &&dot(w,pAb)^2> r^2 * (sum(w.^2))    
                    d3=abs(r-sqrt(sum(w.^2)));
                end
                
                dM=[d1,d2,d3];
                nM=[pAbL/sqrt(sum(pAbL.^2));pAbR/sqrt(sum(pAbR.^2));w/norm(w)];%pAb/pAblen];
                [d,j]=min([d1,d2,d3]);
                v=d*nM(j,:);
                nor=nM(j,:);
                

            else
                w=vRel-pAb;
                d = abs(r - sqrt(sum(w.^2)));
 
                v=d*w/norm(w);
                nor=w/norm(w);

            end
         

           c(end+1)=-1*((agent_vel(1)-(agent_vel0(1)+.5*v(1)))*nor(1)+ (agent_vel(2)-(agent_vel0(2)+.5*v(2)))*nor(2));
 
        end
        
        
        end
        end
        
    end