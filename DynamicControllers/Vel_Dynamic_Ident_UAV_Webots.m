clc
clear all
close all

ts=0.1;
t=[0:ts:10];

%% CONSTANTS VALUES OF THE ROBOT
a = 0.0; 
b = 0.0;
c = 0.0;
L = [a, b, c];

%% 1) PUBLISHER TOPICS & MSG ROS
robot = rospublisher('/Mavic_2_PRO/cmd_vel');
velmsg = rosmessage(robot);
send_velocities(robot, velmsg, [0, 0, 0, 0, 0 , 0]);

%% 2) Suscriber TOPICS & MSG ROS
odom = rossubscriber('/Mavic_2_PRO/odom');

%% 
while 1

continuar=input('PULSA CUALQUIER TECLA PARA CONTINUAR');
break
end 

%% 3) Condiciones iniciles del UAV
[h, hp] = odometry(odom, L);
xu(1) = h(1); 
yu(1) = h(2); 
zu(1) = h(3); 
psi(1)= h(4);

   
w_ref_active = false;
n=11; % Seleccion UAV

%% VELOCIDADES PARA EL UAV
switch n    
    case 1
        % velocidades escalonadas 
        close all
        b=0;
        R=rand(1,6);
        banda=40;
        for k=1:length(t)
    
            if b<=banda
            a(k)=-1;
            b=b+1;
                aux=1;
            end
            if b>banda && b<=2*banda
            a(k)=1;
            aux=2;
            b=b+1;
            end   

            if aux==1
            V(k,:)=abs(R);
            end
            if aux==2
            V(k,:)=-abs(R);
            end
    
            if b>2*banda
            b=0;
            R=rand(1,6);
            end

            ul_ref(k)=1.25*V(k,1);
            um_ref(k)=-1.25*V(k,2);
            un_ref(k)=1.0*V(k,3);
            %w_ref(k)=0.75*V(k,4);

        end
        
        banda_=125;
        for k=1:length(t)
    
            if b<=banda_
            a(k)=-1;
            b=b+1;
                aux=1;
            end
            if b>banda_ && b<=2*banda_
            a(k)=1;
            aux=2;
            b=b+1;
            end   

            if aux==1
            V(k,:)=abs(R);
            end
            if aux==2
            V(k,:)=-abs(R);
            end
    
            if b>2*banda_
            b=0;
            R=rand(1,6);
            end

            %ul_ref(k)=1.25*V(k,1);
            %um_ref(k)=-1.25*V(k,2);
            %un_ref(k)=1.0*V(k,3);
            w_ref(k)=0.75*V(k,4);

        end
            
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref); 
        %%
    case 2
        %velocidades de referncia 1
        close all 
        ul_ref=0.5*sin(1*t).*cos(0.1*t)-0.15*cos(1*t);
        um_ref=0.5*cos(0.5*t)-0.15*sin(0.75*t);       
        un_ref=0.4*sin(0.1*t).*cos(-0.5*t)-0.15*cos(0.1*t); 
        w_ref=0.4*cos(0.5*t)+0.3*sin(0.5*t);
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref);       
        %%
    case 3
        % velocidades de referncia 2
        close all
        ul_ref=0.9*sin(0.5*t).*cos(0.5*t)+0.15*cos(0.5*t);
        um_ref=0.4*cos(t)+0.3*cos(0.5*t);
        un_ref=0.1*sin(0.6*t).*sin(0.5*t)+0.3*cos(0.7*t).*cos(0.4*t);
        w_ref=0.3*sin(0.2*t).*cos(0.4*t)-0.15*cos(t); 
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref);
        %%
    case 4
        % % velocidades de referncia 3
        close all
        ul_ref=0.5*sin(1*t).*cos(0.1*t)-0.15*cos(1*t);
        um_ref=0.4*cos(t)+0.3*cos(0.9*t);
        un_ref=0.5*sin(0.75*t).*cos(0.1*t)-0.15*cos(t);
        w_ref=0.25 *cos(0.5*t)+ 0.25 *sin(0.4*t);
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref); 
          
        %%
    case 5
        %  velocidades de referncia 5
        close all
        ul_ref=0.8*sin(0.1*t).*cos(t)+0.15*cos(0.1*t);
        um_ref=0.8*sin(0.5*t).*cos(0.4*t)-0.15*cos(t);
        un_ref=0.4*sin(0.1*t).*cos(0.5*t)-0.15*cos(0.1*t); 
        w_ref=0.8*sin(0.25*t).*cos(0.4*t)-0.15*cos(t);
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref);
    %%
    case 6
        %  velocidades de referncia 5
        close all
        ul_ref=0.4*sin(0.1*t).*cos(0.5*t)-0.15*cos(0.1*t);
        um_ref=0.4*cos(0.1*t)+0.3*cos(0.4*t);
        un_ref=0.1*sin(0.6*t).*sin(0.5*t)+0.3*cos(0.7*t).*cos(0.4*t);
        w_ref=0.25 *cos(0.5*t)+ 0.25 *sin(0.4*t); 
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref);
    %%
    case 7
        close all
        % velocidades de referncia 2
        ul_ref=0.2*sin(0.75*t).*cos(0.8*t)-0.15*cos(t);
        um_ref=0.4*cos(0.1*t)+0.3*cos(0.4*t);
        un_ref=0.5*sin(t).*sin(0.5*t)+0.3*cos(0.7*t).*cos(0.3*t);
        w_ref=0.7*cos(0.2*t)+0.2*sin(0.4*t);
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref);
        %%
    case 8
        close all
        % % velocidades de referncia 3
        ul_ref=0.2*sin(0.75*t).*cos(0.8*t)-0.15*cos(t);
        um_ref=0.4*cos(0.1*t)+0.3*cos(0.4*t);
        un_ref=0.2*sin(0.5*t).*sin(0.5*t)+0.25*cos(0.4*t).*cos(0.7*t);
        w_ref=cos(0.2*t)+sin(0.4*t);
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref);
        %%
    case 9
        close all
        % velocidades de referncia 4
        ul_ref=0.4*sin(0.1*t).*cos(0.5*t)-0.15*cos(0.1*t);
        um_ref=0.2*cos(t)+0.3*cos(0.5*t);
        un_ref=0.1*sin(0.6*t).*sin(0.5*t)+0.3*cos(0.7*t).*cos(0.4*t);
        w_ref=0.8*sin(0.5*t).*cos(0.4*t)-0.15*cos(t);
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref);
        %%
        
    case 10
        close all
        %velocidades de referncia 1
        ul_ref=0.5*sin(t).*cos(0.5*t);
        um_ref=0.2*cos(t).*cos(0.5*t)+0.25*sin(0.7*t);
        un_ref=0.4*cos(0.1*t)+0.3*sin(0.4*t);
        w_ref=0.5*sin(t).*sin(0.5*t)+0.3*cos(0.7*t).*cos(0.3*t);
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref);
        
     case 11
        close all
        %velocidades de referncia 1
        ul_ref=0.5*(0.25 *cos(0.5*t)+ 0.25 *sin(0.4*t));
        um_ref=0.5*(0.7*cos(0.2*t)+0.2*sin(0.4*t));
        un_ref=0.5*(0.25 *cos(0.5*t)+ 0.25 *sin(0.4*t)); 
        w_ref=0.5*(0.7*cos(0.2*t)+0.2*sin(0.4*t));
        subplot(4,1,1)
        plot(ul_ref);
        subplot(4,1,2)
        plot(um_ref);
        subplot(4,1,3)
        plot(un_ref);
        subplot(4,1,4)
        plot(w_ref);   
        %%
    
        
        %%
    otherwise
        disp("Ninguno de los anteriores");
end
%      

if w_ref_active
    disp("w_ref != 0")   
else  
    w_ref= 0*t;
    disp("w_ref = 0")
end
 
%% Comienza el programa
for k=1:length(t)
    tic
    
%% SEND VALUES OF CONTROL ROBOT
    send_velocities(robot, velmsg, [ul_ref(k), um_ref(k), un_ref(k), 0, 0 , w_ref(k)]);
    
    %% GET VALUES OF DRONE
    [h, hp] = odometry(odom, L);
       
    xu_p(k+1) = hp(1);
    yu_p(k+1) = hp(2);
    zu_p(k+1) = hp(3);
    w(k+1) = hp(4);
    
    xu(k+1) = h(1);
    yu(k+1) = h(2);
    zu(k+1) = h(3);      
    psi(k+1) = h(4);
    while toc<0.1
    end
    
% 3) Tiempo de mÃ¡quina   
     dt(k) = toc;
   dt(k) = toc;

end
%%
send_velocities(robot, velmsg, [0, 0, 0, 0, 0 , 0]);
plot(um_ref)
hold on
plot(yu_p)
close all

subplot(4,1,1)
plot(ul_ref);hold on
plot(xu_p)
subplot(4,1,2)
plot(um_ref);hold on
plot(yu_p)
subplot(4,1,3)
plot(un_ref);hold on
plot(zu_p)
subplot(4,1,4)
plot(w_ref); hold on
plot(w)

%% SAVE DATA
filename = 'Test3.mat';
save(filename)
