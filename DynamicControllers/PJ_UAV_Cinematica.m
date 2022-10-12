%**************************************
%************ SEGUIMIENTO DE TRAYECTORIA ******************
%************* ROBOT MANIPULADOR AÃREO *******************
%**************************************
clc; clear all; close all; warning off % Inicializacion
 
ts = 0.1;       % Tiempo de muestreo
tfin = 50;      % Tiempo de simulaciÃ³n
t = 0:ts:tfin;

 

% 1) Condiciones iniciales del manipulador aÃ©reo
     % a) Posiciones iniciales del UAV
     
       
       
     

% 2) Seleccion de trayectoria deseada del manipulador mÃ³vil
   % 1 ---> Churo
   % 2 ---> Seno
   % 3 ---> 8
   % 4 ---> Silla de montar
     Trayectoria_funciones(3,t); 

disp('Empieza el programa')

% Publisher topic

pub_pos = rospublisher('/Position',"geometry_msgs/Pose");
posmsg = rosmessage(pub_pos);

% Condiciones iniciales
xu(1) = 0; 
yu(1) = 0; 
zu(1) = 1; 
psi(1)= 0;
posmsg.Position.X = xu(1);
posmsg.Position.Y = yu(1);
posmsg.Position.Z = zu(1);  
posmsg.Orientation.Z = psi(1);

%% Suscriber TOPICS & MSG ROS
control_pos = rospublisher('/control',"geometry_msgs/Twist");
run_bool = rospublisher('/Run',"geometry_msgs/Point");

controll = rossubscriber('/control');
runcontrol = rossubscriber('/Run');



%**************************************
%************** CONTROLADOR ********************
%***************************************
xd= 25;
yd= 20;
zd= 15;
psid= 1;


run = 0;
disp("Run controller please")
while run == false 
          
     send(pub_pos,posmsg);        
    
     try
        datocontrol = receive(runcontrol,0.5);
        run = datocontrol.X;
     catch
         run=0;
     end 
    
end
disp("Running ...")

hz = 30;
tiempo = 30;
for k=1:tiempo*hz
    tic
  
% 1) LEY DE CONTROL DEL MANIPULADOR AÃREO 
     
     % a) FunciÃ³n que contiene la ley de control del manipulador aÃ©reo
       % LEY DE CONTROL
    % Read odometry values from ros
    try
        odomdata = receive(controll,0.05);  %(the second argument is a time-out in seconds).
        ul(k) = odomdata.Linear.X;
        um(k) = odomdata.Linear.Y;
        un(k) = odomdata.Linear.Z;
        w(k) = odomdata.Angular.Z;
        
        vctry = [ul(k) um(k) un(k) w(k)];
       
    catch
        ul(k) = 0;
        um(k) = 0;
        un(k) = 0;
        w(k) = 0;
        vccatch = [ul(k) um(k) un(k) w(k)];
    end
    
    
    % Get values of position an orientation
    
%     respuesta_uav = PosUAV(xd,yd,zd,psid,xu(k),yu(k),zu(k),psi(k)); 
%     ul(k) = respuesta_uav(1);
%     um(k) = respuesta_uav(2);
%     un(k) = respuesta_uav(3);
%     w(k)  = respuesta_uav(4); %rad
       
% 2) ROBOT MANIPULADOR AÃREO
    
    % a) CinemÃ¡tica del UAV 
      xu_p(k) = ul(k) * cos(psi(k)) - um(k) * sin(psi(k));
      yu_p(k) = ul(k) * sin(psi(k)) + um(k) * cos(psi(k));
      zu_p(k) = un(k);
    
    % b) MÃ©todo Euler IntegraciÃ³n numerica de las velocidades del UAV
      xu(k+1) = xu_p(k)*ts + xu(k);
      yu(k+1) = yu_p(k)*ts + yu(k);
      zu(k+1) = zu_p(k)*ts + zu(k);      
      psi(k+1) = Angulo(w(k)*ts + psi(k));
      
    posmsg.Position.X = xu(k+1);
    posmsg.Position.Y = yu(k+1);
    posmsg.Position.Z = zu(k+1);  
    posmsg.Orientation.Z = psi(k+1);
    
    send(pub_pos,posmsg);

    
    while toc < (1/hz)
    end
    
% 3) Tiempo de mÃ¡quina   
     dt(k) = toc;
end
disp('Fin de los cÃ¡lculos')

%**************************************
%*********** ANIMACIÃN SEGUIMIENTO DE TRAYECTORIA **************
%% **************************************
disp('SimulaciÃ³n RUN')

% 1) ParÃ¡metros del cuadro de animacion
  figure(1)
  axis equal
  view(-15,15) % Angulo de vista
  cameratoolbar
  title ("SimulaciÃ³n")
  
% 2) Configura escala y color del UAV
 Drone_Parameters(0.02);
 H1 = Drone_Plot_3D(xu(1),yu(1),zu(1),0,0,psi(1));hold on
 

% 3) Dimensiones del brazo robÃ³tico
  

% 4) Dibujo inicial del manipulador aÃ©reo

  % a) GrÃ¡fica inicial del brazo
  
  
  % b) GrÃ¡fica inicial del UAV  

  
  % c) GrÃ¡fica de la trayectoria deseada
  plot3(xd,yd,zd,'--')
  xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
  
% 5) SimulaciÃ³n de movimiento del manipulador aÃ©reo
  for k=1:4:length(t)  
  % a) Eliminas los dibujos anteriores del manipulador aÃ©reo
    delete(H1);
    H1 = Drone_Plot_3D(xu(k),yu(k),zu(k),0,0,psi(k)); hold on
  % b) GrÃ¡fica la posiciÃ³n deseada vs actual en cada frame
    plot3(xu(1:k),yu(1:k),zu(1:k),'r')
    hold on
    plot3(xd,yd,zd,'b')
    
  % c) Grafica de brazo robÃ³tico
    
    
  % d) GrÃ¡fica del UAV
    
  
  pause(0.1)
  end
%%
%**************************************
%*************** GRÃ?FICAS *********************
%% **************************************

% 1) Igualar columnas de los vectores creados
  xu(:,end)=[];
  yu(:,end)=[];
  zu(:,end)=[];
  psi(:,end)=[];
  
% 2) CÃ¡lculos del Error
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
  title ("Errores de posiciÃ³n")
  
% 3) Posiciones deseadas vs posiciones reales del extremo operativo del manipulador aÃ©reo
  figure(3)
  
  subplot(4,1,1)
  plot(xd)
  hold on
  plot(xu)
  legend("xd","hx")
  ylabel('x [m]'); xlabel('s [ms]');
  title ("Posiciones deseadas y reales del extremo operativo del manipulador aÃ©reo")
  
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