function [xa_apo,Pa_apo,debugOutput]...
    = posEKF(zFlag,dt,z,q_acc,q_speed,q_pos,r_acc,r_ptam,r_got)


%% model specific parameters

%% init
persistent x_apo
if(isempty(x_apo))
    acc=single([0;0;0]);
    vel=single([0;0;0]);
    pos=single([0;0;0]);
    x_apo=single([acc;vel;pos]);
end

persistent accx accy accz velx vely velz posx posy posz
if(isempty(posx))
    accx = single(0);
    accy = single(0);
    accz = single(0);
    velx = single(0);
    vely = single(0);
    velz = single(0);
    posx = single(0);
    posy = single(0);
    posz = single(0);
end

persistent P_apo
if(isempty(P_apo))
    P_apo = single(200*ones(9));
end

debugOutput = single(zeros(4,1));

%% copy the states
accx=  x_apo(1);   % x  body acceleration
accy=  x_apo(2);   % y  body acceleration
accz=  x_apo(3);   % z  body acceleration

velx=  x_apo(4);  % x  body velocity
vely=  x_apo(5);  % y  body velocity
velz=  x_apo(6);  % z  body velocity

posx =  x_apo(7);  % x  body position
posy=  x_apo(8);  % y  body position
posz=  x_apo(9);  % z  body positiony


%% prediction section
% compute the apriori state estimate from the previous aposteriori estimate
%body accelerations

    acck =[accx;accy;accz];

%body velocity
    velk =[velx;  vely; velz] + dt*acck;

%body position
    posk = [posx; posy; posz] + dt*velk + 0.5*dt*acck;


x_apr=[acck;velk;posk];

% compute the apriori error covariance estimate from the previous
%aposteriori estimate

E=single(eye(3));
Z=single(zeros(3));

A_lin=[ Z,  Z, Z;...
        E,  Z,  Z;...
        0.5*dt*E, E,  Z];

    
A_lin=eye(9)+A_lin*dt;

%process covariance matrix
persistent Q
if (isempty(Q))
    Q=diag([q_acc,q_acc,q_acc,...
        q_speed,q_speed,q_speed,...
        q_pos,q_pos,q_pos]);
end

P_apr=A_lin*P_apo*A_lin'+Q;

%% update
%% update
if zFlag(1)==1&&zFlag(2)==1&&zFlag(3)==1
    
    %     R=[ r_acc,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    %         0,r_acc,0,0,0,0,0,0,0,0,0,0,0,0,0;
    %         0,0,r_acc,0,0,0,0,0,0,0,0,0,0,0,0;
    %         0,0,0,r_ptam,0,0,0,0,0,0,0,0,0,0,0;
    %         0,0,0,0,r_ptam,0,0,0,0,0,0,0,0,0,0;
    %         0,0,0,0,0,r_ptam,0,0,0,0,0,0,0,0,0;
    %         0,0,0,0,0,0,r_got,0,0,0,0,0,0,0,0;
    %         0,0,0,0,0,0,0,r_got,0,0,0,0,0,0,0;
    %         0,0,0,0,0,0,0,0,r_got,0,0,0,0,0,0;
    R_v=[r_acc,r_acc,r_acc,r_ptam(1),r_ptam(2),r_ptam(3),r_got,r_got,r_got];
    
    H_k=[  E,     Z,      Z; 
           Z,     Z,      E;
           Z,     Z,      E];

    y_k=z(1:9)-H_k*x_apr;
    
    
    %S_k=H_k*P_apr*H_k'+R;
    S_k=H_k*P_apr*H_k';
    S_k(1:9+1:end) = S_k(1:9+1:end) + R_v;
    K_k=(P_apr*H_k'/(S_k));
    
    
    x_apo=x_apr+K_k*y_k;
    P_apo=(eye(9)-K_k*H_k)*P_apr;

    else
        if zFlag(1)==1&&zFlag(2)==0&&zFlag(3)==0
            
    %     R=[ r_acc,0,0;
    %         0,r_acc,0;
    %         0,0,r_acc;
    
            R_v=[r_acc,r_acc,r_acc];
            %observation matrix
            
            H_k=[  E,     Z,      Z;];
            
            y_k=z(1:3)-H_k(1:3,1:9)*x_apr;
            
            S_k=H_k(1:3,1:9)*P_apr*H_k(1:3,1:9)';
            S_k(1:3+1:end) = S_k(1:3+1:end) + R_v;
            K_k=(P_apr*H_k(1:3,1:9)'/(S_k));
            
            
            x_apo=x_apr+K_k*y_k;
            P_apo=(eye(9)-K_k*H_k(1:3,1:9))*P_apr;
        else
            if  zFlag(1)==1&&zFlag(2)==1&&zFlag(3)==0
                
    %     R=[ r_acc,0,0,0,0,0;
    %         0,r_acc,0,0,0,0;
    %         0,0,r_acc,0,0,0;
    %         0,0,0,r_ptam,0,0;
    %         0,0,0,0,r_ptam,0;
    %         0,0,0,0,0,r_ptam;
               
    R_v=[r_acc,r_acc,r_acc,r_ptam(1),r_ptam(2),r_ptam(3)];
                %observation matrix
                
                H_k=[  E,     Z,      Z;
                       Z,     Z,      E];
                
                y_k=z(1:6)-H_k(1:6,1:9)*x_apr;
                
                S_k=H_k(1:6,1:9)*P_apr*H_k(1:6,1:9)';
                S_k(1:6+1:end) = S_k(1:6+1:end) + R_v;
                K_k=(P_apr*H_k(1:6,1:9)'/(S_k));
                
                
                x_apo=x_apr+K_k*y_k;
                P_apo=(eye(9)-K_k*H_k(1:6,1:9))*P_apr;
            else
                if  zFlag(1)==1&&zFlag(2)==0&&zFlag(3)==1
    %        R=[ r_acc,0,0,0,0,0;
    %         0,r_acc,0,0,0,0;
    %         0,0,r_acc,0,0,0;
    %         0,0,0,r_got,0,0;
    %         0,0,0,0,r_got,0;
    %         0,0,0,0,0,r_got;
    R_v=[r_acc,r_acc,r_acc,r_got,r_got,r_got];
                    %observation matrix
                    
                    H_k=[  E,     Z,      Z;
                        Z,     Z,        E];
                    
                    y_k=[z(1:3);z(7:9)]-H_k(1:6,1:9)*x_apr;
                    
                    S_k=H_k(1:6,1:9)*P_apr*H_k(1:6,1:9)';
                    S_k(1:6+1:end) = S_k(1:6+1:end) + R_v;
                    K_k=(P_apr*H_k(1:6,1:9)'/(S_k));
                    
                    
                    x_apo=x_apr+K_k*y_k;
                    P_apo=(eye(9)-K_k*H_k(1:6,1:9))*P_apr;
                else
                    x_apo=x_apr;
                    P_apo=P_apr;
                end
            end
        end
end
xa_apo =x_apo;
Pa_apo =P_apo;
end
