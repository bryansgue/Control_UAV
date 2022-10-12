%******************************************************************************************************************
%************************************ SEGUIMIENTO DE TRAYECTORIA **************************************************
%************************************* ROBOT MANIPULADOR AÉREO *****************************************************
%******************************************************************************************************************
clc; clear all; close all; warning off % Inicializacion

ts = 0.1;       % Tiempo de muestreo
tfin = 100;      % Tiempo de simulación
t = 0:ts:tfin;
a=0;
b=0;

%% CONSTANTS VALUES OF THE ROBOT
a = 0.0; 
b = 0.0;
c = 0.0;
L = [a, b, c];

%% 1) PUBLISHER TOPICS & MSG ROS
robot = rospublisher('/Mavic_2_PRO/cmd_vel');
velmsg = rosmessage(robot);
send_velocities(robot, velmsg, [0, 0, 0, 0, 0 , 0]);

while 1
continuar=input('PULSA CUALQUIER TECLA PARA CONTINUAR');
break
end 

%% 2) Suscriber TOPICS & MSG ROS
odom = rossubscriber('/Mavic_2_PRO/odom');


%% 3) Condiciones iniciles del UAV
[h, v_real] = odometry(odom, L);
xu(1) = h(1); 
yu(1) = h(2); 
zu(1) = h(3); 
psi(1)= h(4);
%% Variables definidas por la TRAYECTORIA y VELOCIDADES deseadas
[xd, yd, zd, psid, xdp, ydp, zdp, psidp] = Trayectorias(3,t);
%% GENERALIZED DESIRED SIGNALS
psid = 0* ones(1,length(t));
psidp = 0* ones(1,length(t));
hd = [xd; yd; zd; psid];
hdp = [xdp;ydp;zdp;psidp];


%% Matriz A & B del sistema dinamico DMD Offline (Continuo)
load("A_B_values.mat"); 

%% Values init for DMD ONLINE (Discreto)
load("G&P_DMDonline_values_init.mat");
Ae = G(:,1:4);
Be = G(:,5:end);

%% Velocidad inicial real del UAV
% v_real = [0;0;0;0];
v_est = v_real(:,1);
sample = 0;
A_E_P = reshape(Ae,16,1)
B_E_P = reshape(Be,16,1) 

A_c = (Ae-eye(4))/ts;
B_c = Be/ts;



%% Windowed DMD Online Value
m=6;

%******************************************************************************************************************
%***************************************** CONTROLADOR ***********************************************************
%*****************************************************************************************************************
disp('Empieza el programa')
for k=1:length(t)-1
tic
%% 1) LEY DE CONTROL
vc(:,k) = Vc_UAV(hdp(:,k),hd(:,k),xu(k),yu(k),zu(k),psi(k)); 
ul(k)=vc(1,k); um(k)=vc(2,k); un(k)=vc(3,k); w(k)=vc(4,k);
vcp = [0;0;0;0];
%% DYAMIC COMPENSATION
vref(:,k) = dynamicComDMD_online(A, B, vcp, vc(:,k), v_real(:,k),0.8, 1);
%% SEND VALUES OF CONTROL ROBOT
send_velocities(robot, velmsg, [vref(1,k), vref(2,k), vref(3,k), 0, 0 ,  vref(4,k)]);
    
%% 2) DINAMICA DEL UAV (VELOCIDAD Y POSICION)

 %% GET VALUES OF DRONE
[h, hp] = odometry(odom, L);

xu_p(k+1) = hp(1);
yu_p(k+1) = hp(2);
zu_p(k+1) = hp(3);
w(k+1) = hp(4);
v_real(:,k+1) = [xu_p(k+1);yu_p(k+1);zu_p(k+1);w(k+1)];

xu(k+1) = h(1);
yu(k+1) = h(2);
zu(k+1) = h(3);      
psi(k+1) = h(4);


%% A and B Estimation DMD ONLINE
A_c = (Ae-eye(4))/ts;
B_c = Be/ts;
vp_est = (A_c*v_est(:,k)+B_c*vref(:,k));
v_est(:, k+1) = v_est(:,k) + vp_est*ts;
if sample >= m
    [Ae,Be,P,G] = DMD_Online(m,v_est,vref,v_real,P,G,k);
    sample = 0;   
end
sample = sample + 1;



%% 3) Tiempo de máquina   

A_E_P(:,k+1) = reshape(Ae,16,1);
B_E_P(:,k+1) = reshape(Be,16,1);

while toc < ts
end

dt(k) = toc;
end
disp('Fin de los calculos')
send_velocities(robot, velmsg, [0, 0, 0, 0, 0 , 0]);
%*************************************************************************%
%**************ANIMACION SEGUIMIENTO DE TRAYECTORIA **********************%
%% ***********************************************************************%
disp('Animacion RUN')

% 1) Parámetros del cuadro de animacion
figure(1)
axis equal
view(-15,15) % Angulo de vista
cameratoolbar
title ("Simulacion")

% 2) Configura escala y color del UAV
Drone_Parameters(0.02);
H1 = Drone_Plot_3D(xu(1),yu(1),zu(1),0,0,psi(1));hold on


% c) Gráfica de la trayectoria deseada
plot3(xd,yd,zd,'--')
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');

% 5) Simulación de movimiento del manipulador aéreo
for k=1:200:length(t)  
% a) Eliminas los dibujos anteriores del manipulador aéreo
delete(H1);
H1 = Drone_Plot_3D(xu(k),yu(k),zu(k),0,0,psi(k)); hold on
% b) Gráfica la posición deseada vs actual en cada frame
plot3(xu(1:k),yu(1:k),zu(1:k),'r')
hold on
plot3(xd(1:k),yd(1:k),zd(1:k),'b')

pause(0.1)
end

disp('FIN Simulación RUN')  

%%
%******************************************************************************************************************
%********************************************* GR�?FICAS ***********************************************************
%% ****************************************************************************************************************


% 2) Cálculos del Error
figure(2)
hxe= xd - xu;
hye= yd - yu;
hze= zd - zu;
psie= Angulo(psid-psi);
plot(hxe), hold on, grid on
plot(hye)
plot(hze)
plot(psie)
legend("hxe","hye","hze","psie")
title ("Errores de posición")

% 3) Posiciones deseadas vs posiciones reales del extremo operativo del manipulador aéreo
figure(3)

subplot(4,1,1)
plot(xd)
hold on
plot(xu)
legend("xd","hx")
ylabel('x [m]'); xlabel('s [ms]');
title ("Posiciones deseadas y reales del extremo operativo del manipulador aéreo")

subplot(4,1,2)
plot(yd)
hold on
plot(yu)
legend("yd","hy")
ylabel('y [m]'); xlabel('s [ms]');

subplot(4,1,3)
plot(zd)
hold on
plot(zu)
grid on
legend("zd","hz")
ylabel('z [m]'); xlabel('s [ms]');

subplot(4,1,4)
plot(Angulo(psid))
hold on
plot(psi)
legend("psid","psi")
ylabel('psi [rad]'); xlabel('s [ms]');

% 3) Posiciones deseadas vs posiciones reales del extremo operativo del manipulador aéreo
figure(4)


plot(vc(1,20:end))
hold on
plot(v_real(1,20:end))
hold on
plot(vref(1,20:end))
hold on
plot(v_est(1,20:end))
legend("ulc","ul","ul_{ref}","ul_{est}")
ylabel('x [m/s]'); xlabel('s [ms]');
title ("Posiciones deseadas y reales del extremo operativo del manipulador aéreo")

figure(5)
plot(vc(2,20:end))
hold on
plot(v_real(2,20:end))
hold on
plot(vref(2,20:end))
hold on
plot(v_est(2,20:end))
legend("umc","um","um_{ref}","um_{est}")
ylabel('y [m/s]'); xlabel('s [ms]');

figure(6)
plot(vc(3,20:end))
hold on
plot(v_real(3,20:end))
hold on
plot(vref(3,20:end))
hold on
plot(v_est(3,20:end))
legend("unc","un","un_{ref}","un_{est}")
ylabel('z [m/ms]'); xlabel('s [ms]');

figure(7)
plot(vc(4,20:end))
hold on
plot(v_real(4,20:end))
hold on
plot(vref(4,20:end))
hold on
plot(v_est(4,20:end))
legend("wc","w","w_{ref}","w_{est}")
ylabel('psi [rad/s]'); xlabel('s [ms]');

figure(8)
plot(A_E_P',"-",'linewidth',2)
% legend("wc","w","w_{ref}")
ylabel('psi [rad/s]'); xlabel('s [ms]');

figure(9)

plot(B_E_P',"-",'linewidth',2)
% legend("wc","w","w_{ref}")
ylabel('psi [rad/s]'); xlabel('s [ms]');



  