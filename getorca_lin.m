function[ v,nor] = getorca_lin(agent,agent_pos,obs_pos,agent_vel0,obs_vel,j,N)

v = zeros(1,2);
nor = zeros(1,2);

c = [];
ceq = [];
tau = 0.8*2 ;    
       % agent_vel = [u(j) u(N+j)];
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
         

         %  c(end+1)=-1*((agent_vel(1)-(agent_vel0(1)+.5*v(1)))*nor(1)+ (agent_vel(2)-(agent_vel0(2)+.5*v(2)))*nor(2));
 
        end
        %{
         a = zeros(1,3*agent.N);     
         a(j) = -nor(1);
         a(N+j) = -nor(2);
         b = -(agent_vel0(1)+v(1)*0.5)*nor(1) -(agent_vel0(2)+v(2)*0.5)*nor(2) ;
        %}
    end
