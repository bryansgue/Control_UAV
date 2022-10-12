% Programa de control predictivo para un drone basado en optimizacion usando Casadi
%% Clear variables
clc, clear all, close all;

load("chi_values.mat");
chi_real = chi';

%% DEFINITION OF TIME VARIABLES
f = 30 % Hz 
ts = 1/f;
to = 0;
tf = 50;
t = (to:ts:tf);

%% Definicion del horizonte de prediccion
N = 10; 

%% CONSTANTS VALUES OF THE ROBOT
a = 0.1; 
b = 0.1;
c = 0.0;
L = [a, b, c];

% Definicion de los estados iniciales del sistema
x(1) = 0;
y(1) = 0;
z(1) = 0;
psi(1) = 0;
h = [x;y;z;psi]

%% INITIAL GENERALIZE VELOCITIES
v = [0; 0;0;0];

%% GENERAL VECTOR DEFINITION
H = [h;v];

%% Variables definidas por la TRAYECTORIA y VELOCIDADES deseadas
[hxd, hyd, hzd, hpsid, hxdp, hydp, hzdp, hpsidp] = Trayectorias(3,t);

%% GENERALIZED DESIRED SIGNALS
%hd = [hxd; hyd; hzd; hpsid];
hd = [hxd;hyd;hzd;hpsid;hxdp; hydp; hzdp; hpsidp];

%hdp = [hxdp;hydp;hzdp;hpsidp];

%% Deficion de la matriz de la matriz de control
Q = 0.9*eye(4);

%% Definicion de la matriz de las acciones de control
R = 0.001*eye(4);

%% Definicion de los limites de las acciondes de control
bounded = [1.2; -1.2; 1.2; -1.2; 1.2; -1.2; 1; -1];

%% Definicion del vectro de control inicial del sistema
vcc = zeros(N,4);
H0 = repmat(H,1,N+1)'; 

% Definicion del optimizador
[f, solver, args] = mpc_drone(chi_real,bounded, N, L, ts, Q, R);

% Chi estimado iniciales
chi_estimados(:,1) = chi';
tic
for k=1:length(t)-N


    %% Generacion del; vector de error del sistema
    he(:,k)=hd(1:4,k)-h(:,k);
    
    args.p(1:8) = [h(:,k);v(:,k)]; % Generacion del estado del sistema
    
    for i = 1:N % z
        args.p(8*i+1:8*i+8)=[hd(:,k+i)];
%         args.p(4*i+5:4*i+7)=obs;
    end 
    
    args.x0 = [reshape(H0',8*(N+1),1);reshape(vcc',size(vcc,2)*N,1)]; % initial value of the optimization variables
    tic;
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    toc
    sample(k)=toc;
    opti = reshape(full(sol.x(8*(N+1)+1:end))',4,N)';
    H0 = reshape(full(sol.x(1:8*(N+1)))',8,N+1)';
    hfut(:,1:4,k+1) = H0(:,1:4);
    vc(:,k)= opti(1,:)';
    
    vcp(:,k) = [0;0;0;0];
    %% Compensador Dinamico Adaptativo
    [Test(:,k),chi_estimados(:,k+1)] = estimadaptive_dymanic_UAV(chi_estimados(:,k),vcp(:,k), vc(:,k), v(:,k), hd(1:4,k), h(:,k) ,1, L, ts);
    vref(:,k)= vc(:,k)+Test(:,k);
    
    %% Dinamica del sistema 
    [v(:, k+1),Tu(:,k)] = dyn_model_adapUAV(chi_real, v(:,k), vref(:,k), psi(k), L,ts,k);
    
    % CHANGE DYNAMIC PARAMETETS
    
%     v(:, k+1) = dyn_model_UAV(chi_real(:,k), v(:,k), vref(:,k), psi(k), L,ts);
%     minimo = -0.0332;
%     maximo =  0.03821;
%     noise(:,k)  =  (maximo-minimo) .* rand(19,1) + minimo;
%     chi_real(:,k+1) = chi_real(:,k)  + noise(:,k);
%     Tu(:,k) = [0 0 0 0]';
    
    %% Simulacion del sistema 
    h(:,k+1) = h(:,k)+ UAV_RK4(h(:,k),v(:,k+1),ts);
    hx(k+1) = h(1,k+1);
    hy(k+1) = h(2,k+1);
    hz(k+1) = h(3,k+1);      
    psi(k+1) = Angulo(h(4,k+1));
    
        
    %% Actualizacion de los resultados del optimizador para tener una soluciona aproximada a la optima
    
    vcc = [opti(2:end,:);opti(end,:)];
    H0 = [H0(2:end,:);H0(end,:)];
end
toc
%%
close all; paso=1; 
%a) Parámetros del cuadro de animación
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
    set(gcf, 'PaperPosition', [0 0 8 3]);
    h = light;
    h.Color=[0.65,0.65,0.65];
    h.Style = 'infinite';
%b) Dimenciones del Robot
    Drone_Parameters(0.02);
%c) Dibujo del Robot    
    G2=Drone_Plot_3D(hx(1),hy(1),hz(1),0,0,psi(1));hold on

    G3 = plot3(hx(1),hy(1),hz(1),'-','Color',[56,171,217]/255,'linewidth',1.5);hold on,grid on   
    G4 = plot3(hxd(1),hyd(1),hzd(1),'Color',[32,185,29]/255,'linewidth',1.5);
    G5 = Drone_Plot_3D(hx(1),hy(1),hz(1),0,0,psi(1));hold on
%    plot3(hxd(ubicacion),hyd(ubicacion),hzd(ubicacion),'*r','linewidth',1.5);
    view(20,15);

for k = 1:10:length(t)-N
    %drawnow
    delete(G2);
    delete(G3);
    delete(G4);
    delete(G5);
   
    G2=Drone_Plot_3D(hx(k),hy(k),hz(k),0,0,psi(k));hold on  
    G3 = plot3(hxd(1:k),hyd(1:k),hzd(1:k),'Color',[32,185,29]/255,'linewidth',1.5);
    G4 = plot3(hx(1:k),hy(1:k),hz(1:k),'-.','Color',[56,171,217]/255,'linewidth',1.5);
    G5 = plot3(hfut(1:N,1,k),hfut(1:N,2,k),hfut(1:N,3,k),'Color',[100,100,100]/255,'linewidth',0.1);

    pause(0)
end

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);

plot(t(1:length(he)),he(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t(1:length(he)),he(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on;
plot(t(1:length(he)),he(3,:),'Color',[26,115,160]/255,'linewidth',1);hold on;
plot(t(1:length(he)),he(4,:),'Color',[83,57,217]/255,'linewidth',1);hold on;
grid on;
legend({'$\tilde{h_{x}}$','$\tilde{h_{y}}$','$\tilde{h_{z}}$','$\tilde{h_{\psi}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Evolution of Control Errors}$','Interpreter','latex','FontSize',9);
ylabel('$[m]$','Interpreter','latex','FontSize',9);

figure

subplot(4,1,1)
plot(Tu(1,:))
hold on
plot(Test(1,:))
legend("Tx_u","Tx_{est}")
ylabel('x [m]'); xlabel('s [ms]');
title('$\textrm{Evolution of h }$','Interpreter','latex','FontSize',9);

subplot(4,1,2)
plot(Tu(2,:))
hold on
plot(Test(2,:))
legend("Ty_u","Ty_{est}")
ylabel('y [m]'); xlabel('s [ms]');

subplot(4,1,3)
plot(Tu(3,:))
hold on
plot(Test(3,:))
grid on
legend("Tz_u","Tz_{est}")
ylabel('z [m]'); xlabel('s [ms]');

subplot(4,1,4)
plot(Tu(4,:))
hold on
plot(Test(4,:))
legend("Tpsi_u","Tpsi_{est}")
ylabel('psi [rad]'); xlabel('s [ms]');
%% %%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);

plot(t(1:length(vc)),vc(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t(1:length(vc)),vc(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on;
plot(t(1:length(vc)),vc(3,:),'Color',[26,115,160]/255,'linewidth',1);hold on;
plot(t(1:length(vc)),vc(4,:),'Color',[83,57,217]/255,'linewidth',1);hold on;
grid on;
legend({'${v_{cx}}$','${v_{cy}}$','${v_{cz}}$','${v_{c\psi}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Acciones de control optimas}$','Interpreter','latex','FontSize',9);
ylabel('$[m]$','Interpreter','latex','FontSize',9);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);

plot(t(1:length(vref)),vref(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t(1:length(vref)),vref(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on;
plot(t(1:length(vref)),vref(3,:),'Color',[26,115,160]/255,'linewidth',1);hold on;
plot(t(1:length(vref)),vref(4,:),'Color',[83,57,217]/255,'linewidth',1);hold on;
grid on;
legend({'${v_{refx}}$','${v_{refy}}$','${v_{refz}}$','${v_{ref\psi}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Acciones de control compensadas}$','Interpreter','latex','FontSize',9);
ylabel('$[m]$','Interpreter','latex','FontSize',9);
%%                 

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(sample)),sample,'Color',[46,188,89]/255,'linewidth',1); hold on
grid on;
legend({'$t_{sample}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Sample Time}$','Interpreter','latex','FontSize',9);
ylabel('$[s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

%%
figure
plot(chi_estimados',"-",'linewidth',2)
ylabel('psi [rad/s]'); xlabel('s [ms]');

figure
plot(chi_real',"-",'linewidth',2)
ylabel('psi [rad/s]'); xlabel('s [ms]');