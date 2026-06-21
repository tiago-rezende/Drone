Dado um quadrotor simples, como mostrado na figura:



Faremos o primeiro controle para a simulação, para poder introduzir ao campo de Drones. Para tanto, faremos a suposição de que a estrutura é feita de 4 cubóides de tamanho L x W x H mais um cubóide de W x W x H. 
massa motor = 25g (cada)
massa bateria = 250
tipo de helice = 
Seleção de parâmetros

pho = 1.6; %Densidade do material em g/cm^2
L = 30; %Comprimento do braço em cm
W =2; %Largura do braço em cm
H =0.2; %Altura do Braço em cm

Suposições e construção dos dados
A partir dessas insformações podemos calcular o volume, massa e momentos de inércia do frame do quadrotor. Além disso, mais algumas suposiçõies:
O motor, juntamente com a hélice, possui 10% da massa do frame.
Trate o motor com hélice como um cilindro oco, com 80% da massa, e a hélice, como um disco, sem espessura.
O diâmetro do motor deve ser igual a largura W do braço.
O diâmetro da hélice pode ser definida como 6 vezes o diâmetro do motor.
Desconsidere as distâncias do frame ao motor e do motor a hélice, para o cálculo da inércia (sem eixos paralelos entre as partes).
A partir da escolha de um valor de velocidade de Pairo (Hover), escolher um , tal que a força de 4 motores seja o suficiente para levantar o drone.
A constante  é 10x menor que .
volume_arm_short = L*H*W; %volume em cm^3
mass_s_arm = pho*volume_arm_short/1000; %massa, em kg.
volume_arm_long = (2*L+W)*H*W; %volume em cm^3
mass_l_arm = pho*volume_arm_long/1000; %massa, em kg.
frame_mass = mass_l_arm + 2*mass_s_arm; %massa do frame, em kg
motor_mass = 0.1*frame_mass; %massa do motor, em kg
overall_mass = frame_mass+4*motor_mass; %massa total, em kg
Vamos dividir o braço em 3 partes, a primeira será o braço com o eixo x, de tamanho 2L+W, e 2 pedaços, na direção de y, de tamanho L, que estão deslocados do centro de giro de (L+W)/2.
% Braço longo
Ixx = (mass_l_arm/12)*((W/100)^2+(H/100)^2);  %kg.m^2
Iyy = (mass_l_arm/12)*(((2*L+W)/100)^2+(H/100)^2); %kg.m^2
Izz = (mass_l_arm/12)*(((2*L+W)/100)^2+(W/100)^2); %kg.m^2

%Braços curtos
Ixx = Ixx + 2*((mass_s_arm/12)*((L/100)^2+(H/100)^2)+mass_s_arm*((L+W)/200)^2);
Iyy = Iyy + 2*((mass_s_arm/12)*((W/100)^2+(H/100)^2));
Izz = Izz + 2*((mass_s_arm/12)*((L/100)^2+(W/100)^2)+mass_s_arm*((L+W)/200)^2);

%Motor
Ixx_motor = 0.5*motor_mass*0.8*(W/200)^2+0.25*0.2*motor_mass*(3*W/100)^2; %Ixx do motor com hélice
Iyy_motor = Ixx_motor;
Izz_motor = motor_mass*0.8*(W/200)^2+0.5*0.2*motor_mass*(3*W/100)^2; %Izz do motor com hélice

%overall
Ixx = Ixx + 4*Ixx_motor + 2*motor_mass*((L+W)/200)^2; %Apenas 2 motores passam pelos eixos paralelos.
Iyy = Iyy + 4*Iyy_motor + 2*motor_mass*((L+W)/200)^2; %Apenas 2 motores passam pelos eixos paralelos.
Izz = Izz + 4*(Izz_motor+motor_mass*((L+W)/200)^2);

%tensor de inercia
I_til = [Ixx, 0, 0; 0, Iyy, 0; 0, 0, Izz];
Definições da hélice
w_hover =1000; %em rad/s
g = 9.81; %m/s^2
k_aero = overall_mass*g/(4*w_hover^2);
k_drag = k_aero/10;

quad = struct("Inertia", I_til, "mass", overall_mass, "Prop", Izz_motor, "grav", g, "k_aero", k_aero, "q_aero", k_drag, "Length", L/100, "w_hover", w_hover);

Construção da simulação
O modelo a ser simulado foi o desenvolvido em sala, portanto:


Façamos
line([0, 0],[0, 0],[0,10],'LineWidth',2,'LineStyle','--','Color','b');
hold on;
line([0, -5],[0, 0],[10,10],'LineWidth',2,'LineStyle','--','Color','b');
line([-5, -5],[0, -5],[10,10],'LineWidth',2,'LineStyle','--','Color','b');
line([-5, 5],[-5, -5],[10,10],'LineWidth',2,'LineStyle','--','Color','b');
line([5, 5],[-5, 5],[10,10],'LineWidth',2,'LineStyle','--','Color','b');
line([5, -5],[5, 5],[10,10],'LineWidth',2,'LineStyle','--','Color','b');
line([-5, -5],[5, 0],[10,10],'LineWidth',2,'LineStyle','--','Color','b');
line([-5, 0],[0, 0],[10,0],'LineWidth',2,'LineStyle','--','Color','r');
view(3);
grid on
hold off
Waypoint_list = [0,0,0; 0,0,-10;-5,0,-10;-5,-5,-10;5,-5,-10;5,5,-10;-5,5,-10;-5,0,-10;0,0,0]
dist = 0;
for i=2:size(Waypoint_list,1)
    dist = dist +  sqrt(sum((Waypoint_list(i,:)-Waypoint_list(i-1,:)).^2));
end
disp(dist)
Supondo 1 m/s em todo o trajeto, dado a distancia de 66.1803, o tempo deverá ser de mesmo valor, logo deixaremos o tempo final como 67.
dt=0.001; %Passo de simulação
tfinal = 67;
Iremos iniciar as variáveis, para usar o runge-kutta 4-5, o Matlab sugere quebrar o sistema de segunda ordem em 2 de primeira ordem, portanto, faremos um vetor de 12 estados, sendo 6 deles, apenas a indicação da derivada.
Assim:
 e 

Além disso, definiremos a matriz de rotação, que descreve a mudança do sistema coordenado do corpo para o inercial:

E a matriz de transformação das velocidades angulares para forças e torques, ou seja, usaremos a notação matricial para consolidar os  e :

As funções para a attitude são:
 e 
Vejamos como fica o controle para 100 Hz


Ts = 0.01; %100 Hz
Fk = [1 0 0 0;
    Ts 1 0 0;
    0 0 1 0;
    0 0 Ts 1];
Gk = [Ts/Ixx 0;
    Ts^2/Ixx 0;
    0 Ts/Iyy
    0 Ts^2/Iyy];
Hk = [0 1 0 0;
    0 0 0 1];
ss_ol = ss(Fk, Gk, Hk, [0 0;0 0], Ts);
[b1,a1] = ss2tf(Fk,Gk,Hk,[0 0;0 0],1);
Gphi = tf(b1(1,:),a1,Ts);
[b1,a1] = ss2tf(Fk,Gk,Hk,[0 0;0 0],2);
Gtheta = tf(b1(2,:),a1,Ts);
rlocus(Gphi);
grid on
figure;
rlocus(Gtheta)
grid on

Faremos o controlador via o espaço de estados com a realimentação total dos estados, supondo que temos os dados da IMU e o resultado de seu filtro.
São 4 raízes, portanto, podemos ecolher um desempenho de um sistema de segunda ordem para um eixo e replicar para o outro. Lembrando que:

É a função transferência de um sistema típico de segunda ordem. Lembrando ainda que:
ou 
 e 
Dado que desejamos que o angulo esteja em sua referência dentro de um mesmo loop, podemos supor um tempo de acomodação de 1% de 100 ms e um tempo de pico de 50 ms. Logo,
 e 
Antes de validarmos o pólo, nos certifiquemos que até quanto será possível cumprir esse requisito. Supondo o dobro da velocidade de hover como a máxima velocidade para o cálculo da força, teremos:

T_max = 4*k_aero*w_hover^2
tau_max = (L/100)*T_max
Fazendo um degrau de , temos:
step(Gtheta*tau_max*180/pi,0:0.01:0.1);
Vê-se que, em 50 ms, o drone poderia chegar a 7º, sendo esse o maior valor de tempo de pico para ser verossímel e, em 100 ms, 30º, que é um ângulo muito maior que o necessário.
Passemos, agora, para controle digital:

Ts = 0.01;
ts1pc = 0.11;
tpic = 0.09;
sigma = 4.6/ts1pc;
wd = pi/tpic;
p1 = -sigma + 1i*wd;
p2 = -sigma - 1i*wd;
z1 = exp(Ts*p1);
z2 = exp(Ts*p2);
Façamos a realimentação por alocação de pólos
K = place(Fk,Gk,[z1,z2, z1, z2])

X0 = [0;deg2rad(1);0;deg2rad(1)];
phi = Fk-Gk*K;
cl_os = ss(phi, zeros(4,2),Hk,zeros(2), Ts)
Para colocar a referencia, podemos dividir o calculo em duas partes, uma vez que o espaço de estados está desacoplado.
augmented_matrix_phi = [Fk(1:2,1:2)-eye(2) Gk(1:2,1);Hk(1,1:2) 0];
Nx_u_phi = augmented_matrix_phi\[0;0;1];
Nbar_phi =  Nx_u_phi(3) + K(1,1:2)*Nx_u_phi(1:2);
augmented_matrix_theta = [Fk(3:4,3:4)-eye(2) Gk(3:4,2);Hk(2,3:4) 0];
Nx_u_theta = augmented_matrix_theta\[0;0;1];
Nbar_theta =  Nx_u_theta(3) + K(2,3:4)*Nx_u_theta(1:2);
cl_os_wref = ss(phi, Gk*[Nbar_phi 0;0 Nbar_theta],Hk,zeros(2,2), Ts)
tempo_ctrl = 0:Ts:0.3;
[y,tOut, x_states] = initial(cl_os,X0,tempo_ctrl);
u_ss = -K*x_states';
figure;
subplot(1,2,1);
stairs(tOut, rad2deg(y(:,1)));
subplot(1,2,2);
stairs(tOut, u_ss(1,:));

ref = repmat([deg2rad(1);0], 1, length(tempo_ctrl));
[y_ref,tOut,x_ref] = lsim(cl_os_wref,ref,tempo_ctrl);
u_ref = -K*x_ref'+[Nbar_phi 0;0 Nbar_theta]*ref;
figure;
subplot(1,3,1);
stairs(tOut, rad2deg(y_ref(:,1)));
subplot(1,3,2);
stairs(tOut, rad2deg(y_ref(:,2)));
subplot(1,3,3);
stairs(tOut, u_ref(1,:));
A arquitetura geral é


Façamos a análise para a posição Z e para a orientação, que ocorre a cada 10 Hz.
      e        
Gz = tf(1/overall_mass, [1, 0, 0]);
Gpsi = tf(1/Izz, [1, 0, 0]);
Gzd = c2d(Gz, 0.1);
Gpsid = c2d(Gpsi, 0.1);
Substituindo e discretizando a 100 ms:
   e   
Olhando para o lugar das raízes
figure;
rlocus(Gzd)
Façamos a mesma definição
 e 
Se desejarmos que o tempo de estabilização seja em 1 segundo e o tempo de pico em 0.9 segundos teremos:
 e 
O PD discreto (com a Euler para trás) fica: 

[zz, pp, kk] = zpkdata(Gzd);
ts1z = 1;
tpz = 0.9;
desired_pole_z = -(4.6/ts1z) + 1i*(pi/tpz);
dd_pole = exp(0.1*desired_pole_z);
pc = 0;
phase = -atan2(imag(dd_pole), real(dd_pole)-pc);
gain_c = abs(dd_pole-pc);
for it=1:length(zz{1})
    phase = phase + atan2(imag(dd_pole)-imag(zz{1}(it)), real(dd_pole)-real(zz{1}(it)));
    gain_c = gain_c/abs(dd_pole-(zz{1}(it)));
end
for it=1:length(pp{1})
    phase = phase - atan2(imag(dd_pole)-imag(pp{1}(it)), real(dd_pole)-real(pp{1}(it)));
    gain_c = gain_c*abs(dd_pole-pp{1}(it));
end
phase = pi - phase; %Assumindo sempre +180 e tentar corrigir depois)
while(phase<-pi || phase>pi)
    if(phase>pi)
        phase = phase - 2*pi;
    else
        phase = phase + 2*pi;
    end
end

zc = real(dd_pole) - imag(dd_pole)/(tan(phase)); %Se fosse o polo seria tan(-phase);
gain_c = gain_c/(abs(dd_pole - zc)*kk);
Cz = tf(gain_c*[1, -zc],[1, 0], 0.1)

Fechando a malha fechada:
CL_Z = feedback(Cz*Gzd,1)
%%%% plan traj
pw = 2;
coeffs = zeros(4,3);
t_seg = sqrt(sum((Waypoint_list(pw,:)-Waypoint_list(pw-1,:)).^2)); %assumindo 1 m/s
coeffs(1, 3) = Waypoint_list(pw-1,3);
coeffs(2, 3) = 10*(Waypoint_list(pw,3)-Waypoint_list(pw-1,3))/t_seg^3;
coeffs(3, 3) = 15*(-Waypoint_list(pw,3)+Waypoint_list(pw-1,3))/t_seg^4;
coeffs(4, 3) = 6*(Waypoint_list(pw,3)-Waypoint_list(pw-1,3))/t_seg^5;
plan_time = 0:0.1:10;
des_pos_z = coeffs(:,3)'*[ones(1,length(plan_time)); plan_time.^3; plan_time.^4; plan_time.^5];
% des_vel_z = coeffs(:,3)'*[0; 3*plan_time.^2; 4*plan_time.^3; 5*plan_time.^4];
% des_acc_z = coeffs(:,3)'*[0; 6*plan_time; 12*plan_time.^2; 20*plan_time.^3];
[y_z,tOuty,x_z] = lsim(CL_Z,des_pos_z,plan_time);
U_Rerf = Cz/(1+Cz*Gzd)
[u_z,tOutu,ux_z] = lsim(U_Rerf,des_pos_z,plan_time);
figure;
subplot(1,2,1);
plot(tOuty, des_pos_z, 'b');
hold on;
stairs(tOuty, y_z, 'r');
hold off
subplot(1,2,2);
stairs(tOutu, u_z);

figure;
rlocus(Gpsid)
Façamos a mesma definição
 e 
Se desejarmos que o tempo de estabilização seja em 1 segundo e o tempo de pico em 0.9 segundos teremos:
 e 
O PD discreto (com a Euler para trás) fica: 

[zz, pp, kk] = zpkdata(Gpsid);
ts1z = 1;
tpz = 0.9;
desired_pole_z = -(4.6/ts1z) + 1i*(pi/tpz);
dd_pole = exp(0.1*desired_pole_z);
pc = 0;
phase = -atan2(imag(dd_pole), real(dd_pole)-pc);
gain_c = abs(dd_pole-pc);
for it=1:length(zz{1})
    phase = phase + atan2(imag(dd_pole)-imag(zz{1}(it)), real(dd_pole)-real(zz{1}(it)));
    gain_c = gain_c/abs(dd_pole-(zz{1}(it)));
end
for it=1:length(pp{1})
    phase = phase - atan2(imag(dd_pole)-imag(pp{1}(it)), real(dd_pole)-real(pp{1}(it)));
    gain_c = gain_c*abs(dd_pole-pp{1}(it));
end
phase = pi - phase; %Assumindo sempre +180 e tentar corrigir depois)
while(phase<-pi || phase>pi)
    if(phase>pi)
        phase = phase - 2*pi;
    else
        phase = phase + 2*pi;
    end
end

zc = real(dd_pole) - imag(dd_pole)/(tan(phase)); %Se fosse o polo seria tan(-phase);
gain_c = gain_c/(abs(dd_pole - zc)*kk);
Cpsi = tf(gain_c*[1, -zc],[1, 0], 0.1)
A ultima parte do querba-cabeças é resolver o x e o y
Para essa parte, definimos que a dinâmica do erro deveria seguir:

A dinâmica de um sistema de segunda ordem é:

Logo:


Se definirmos um tempo de pico de 1s e tempo de estabilização em 2 s, teremos:
 e 



t_pico_x = 1;
t_set_x = 2;
wn_des = sqrt((pi/t_pico_x)^2+(4/t_set_x)^2);
zeta_des = 4/(wn_des*t_set_x);
Kd_x = 2*zeta_des*wn_des;
Kp_x = wn_des^2;
Kd_y = Kd_x;
Kp_y = Kp_x;

linear = [0;0;0;0;0;0];
linear_dot = [0;0;0;0;0;0];
zeta = [0;0;0;0;0;0];
zeta_dot = [0;0;0;0;0;0];
states = [linear;zeta];
states_dot = [linear_dot; zeta_dot];
rotZYX = eul2rotm(zeta(1:3)');
omegas = [w_hover; w_hover; w_hover; w_hover];
Omega_r = omegas(1)+omegas(3)-omegas(2)-omegas(4);


A simulação no tempo fica:
tempo = 0:dt:tfinal;
resultados = zeros(12,length(tempo));
desired = zeros(6, length(tempo));
method = 2;
ctrl_states_100 = [0;0;0;0];
pw = 2;
coeffs = zeros(4,3);
t_seg = sqrt(sum((Waypoint_list(pw,:)-Waypoint_list(pw-1,:)).^2)); %assumindo 1 m/s
for ptj=1:3
    coeffs(1, ptj) = Waypoint_list(pw-1,ptj);
    coeffs(2, ptj) = 10*(Waypoint_list(pw,ptj)-Waypoint_list(pw-1,ptj))/t_seg^3;
    coeffs(3, ptj) = 15*(-Waypoint_list(pw,ptj)+Waypoint_list(pw-1,ptj))/t_seg^4;
    coeffs(4, ptj) = 6*(Waypoint_list(pw,ptj)-Waypoint_list(pw-1,ptj))/t_seg^5;
end
done = 0;
ptj_time = 0;
des_pos = coeffs'*[1; ptj_time^3; ptj_time^4; ptj_time^5];
des_vel = coeffs'*[0; 3*ptj_time^2; 4*ptj_time^3; 5*ptj_time^4];
des_acc = coeffs'*[0; 6*ptj_time; 12*ptj_time^2; 20*ptj_time^3];
error = 0;
errork1 = 0;
e_psi = 0;
e_psik1 = 0;

[zzc, ppz, kkz] = zpkdata(Cz);
zero_PDZ = zzc{1};
Kz = kkz;

[zzc, ppz, kkz] = zpkdata(Cpsi);
Kpsi = kkz;
zero_psi = zzc{1};
theta_ref = 0;
phi_ref = 0;
larm = L/100;

global acceleration_auxiliar;
acceleration_auxiliar = [0;0;0];

global angle_auxiliar;
angle_auxiliar = [0;0;0];

global vel_auxiliar;
vel_auxiliar = [0;0;0];

for i= 1:length(tempo) %0:dt:5*sT, 0 é a condição inicial
    resultados(:,i) = states;
    estimated_states = sensors(states,dt);
    desired(:,i) = [des_pos;0;theta_ref;phi_ref];
    if(i==60000)
        disp('DEBUG');
    end

    if(~mod(i-1,10))
        %100 Hz - attitude control
        ctrl_states_100 = [estimated_states(12);estimated_states(9);estimated_states(11);estimated_states(8)];
        u_tauxy = -K*ctrl_states_100+[Nbar_phi 0;0 Nbar_theta]*[phi_ref;theta_ref];
    end
    if(~mod(i-1,100))
        %10 Hz - position control
        error = des_pos(3) - estimated_states(3);
        u_T = Kz*(error - zero_PDZ*errork1);
        errork1=error;
        e_psi = 0 - estimated_states(7);
        u_psi = Kpsi*(e_psi - zero_psi*e_psik1);
        e_psik1=e_psi;
        %%%%
        xdd = des_acc(1)+Kd_x*(des_vel(1)-estimated_states(4))+Kp_x*(des_pos(1)-estimated_states(1));
        ydd = des_acc(2)+Kd_y*(des_vel(2)-estimated_states(5))+Kp_x*(des_pos(2)-estimated_states(2));
        phi_ref = (-1/g)*(xdd*sin(round(estimated_states(7),5))-ydd*cos(round(estimated_states(7),5)));
        theta_ref = (-1/g)*(xdd*cos(round(estimated_states(7),5))+ydd*sin(round(estimated_states(7),5)));
    end
    
    error_dist = sqrt(sum((Waypoint_list(pw,:)-estimated_states(1:3)').^2));

    if(error_dist < 0.005 && ptj_time>=t_seg)
        pw=pw+1;
        ptj_time = 0;
        if(pw<=size(Waypoint_list,1))
            t_seg = sqrt(sum((Waypoint_list(pw,:)-Waypoint_list(pw-1,:)).^2)); %assumindo 1 m/s
            for ptj=1:3
                coeffs(1, ptj) = Waypoint_list(pw-1,ptj);
                coeffs(2, ptj) = 10*(Waypoint_list(pw,ptj)-Waypoint_list(pw-1,ptj))/t_seg^3;
                coeffs(3, ptj) = 15*(-Waypoint_list(pw,ptj)+Waypoint_list(pw-1,ptj))/t_seg^4;
                coeffs(4, ptj) = 6*(Waypoint_list(pw,ptj)-Waypoint_list(pw-1,ptj))/t_seg^5;
            end
        else
            done =1;
            break;
        end
    else
        ptj_time = ptj_time + dt;
        if(ptj_time<=t_seg)
            des_pos = coeffs'*[1; ptj_time^3; ptj_time^4; ptj_time^5];
            des_vel = coeffs'*[0; 3*ptj_time^2; 4*ptj_time^3; 5*ptj_time^4];
            des_acc = coeffs'*[0; 6*ptj_time; 12*ptj_time^2; 20*ptj_time^3];
        else
            des_pos = Waypoint_list(pw,:)';
            des_vel = [0;0;0];
            des_accel = [0;0;0];
        end
    end
        

    tau_x_des = u_tauxy(1);
    tau_y_des = u_tauxy(2);
    T_des = u_T-4*k_aero*w_hover^2;
    tau_z_des = u_psi;

    des_w2_motor = [-1/(4*k_aero), 0, 1/(2*larm*k_aero), -1/(4*k_drag);
                   -1/(4*k_aero), -1/(2*larm*k_aero), 0, 1/(4*k_drag);
                   -1/(4*k_aero), 0, -1/(2*larm*k_aero),-1/(4*k_drag);
                   -1/(4*k_aero), 1/(2*larm*k_aero),0, 1/(4*k_drag)
                   ]*[T_des; tau_x_des; tau_y_des; tau_z_des];

    states = rk4_step(@modelo_quad, tempo(i), states, dt, method, quad, des_w2_motor);
end

Visualização

dT_visualiza = 100;
Motors_body = [L/100 0 -L/100 0;0 L/100 0 -L/100;0 0 0 0];
for i=1:dT_visualiza:size(resultados,2)
    cgPos = resultados(1:3,i);
    R = eul2rotm(resultados(7:9,i)');
    posMotor = cgPos + R*Motors_body; 
    figure(20);
    scatter3(cgPos(1), cgPos(2), cgPos(3), 100, "blue", 'MarkerFaceColor','flat');
    view([300 50]);
    hold on; 
    line('XData',[posMotor(1,1) posMotor(1,3)],'YData',[posMotor(2,1) posMotor(2,3)],'ZData',[-posMotor(3,1) -posMotor(3,3)],'Marker','o','MarkerSize',6,'LineStyle','-','Color','b','LineWidth',2);
    line('XData',[posMotor(1,2) posMotor(1,4)],'YData',[posMotor(2,2) posMotor(2,4)],'ZData',[-posMotor(3,2) -posMotor(3,4)],'Marker','o','MarkerSize',6,'LineStyle','-','Color','g','LineWidth',2);
    grid on;
    xlim([-6 6]);
    xticks(-6:0.5:6);
    ylim([-6 6]);
    yticks(-6:0.5:6);
    
    zlim([0 15])
    zticks(0:0.5:15);
    title(strcat('t =',num2str(tempo(i))));
    pause(0.01);
    hold off;
end

function sensors_values = sensors(states,dt)
gyro_values = giro(states(10:12,1));
accelerometer_values = acelerometro(states(4:6,1),dt);
barometer_values = barometer(states(3,1));
mag_values = magnetonomer(states(7:9,1));
gps_values = gps(states(1:3,1));

global vel_auxiliar;
% x y z
sensors_values(1:2,1) = gps_values(1:2,1);
sensors_values(3,1) = barometer_values;
%x' y' z'
sensors_values(4:5,1) = (gps_values(1:2,1)-vel_auxiliar(1:2,1))/dt;
vel_auxiliar(1:2,1) = gps_values(1:2,1);
sensors_values(6,1) = (barometer_values-vel_auxiliar(3,1))/dt;
vel_auxiliar(3,1) = sensors_values(6,1);

%psi theta phi
rotZYX = eul2rotm(states(7:9,1)')';
global angle_auxiliar;
angle_gyro = gyro_values*dt + angle_auxiliar;
estimated_z = rotZYX*accelerometer_values/norm(accelerometer_values);
estimated_x = mag_values/norm(mag_values);
estimated_y = cross(estimated_z,estimated_x);
estimated_rot = [estimated_x';estimated_y';estimated_z'];
estimated_angles_grav_acc = rotm2eul(estimated_rot)';
filtro_complementar = 0.001;
sensors_values(7:9,1) = (1-filtro_complementar)*angle_gyro +filtro_complementar * estimated_angles_grav_acc;
angle_auxiliar = sensors_values(7:9,1);

%psi' theta' phi'
sensors_values(10:12,1) = gyro_values;


end

function valores_giroscopio = giro(angular_velocities)
    desvio_padrao_ruido = 0;%0.1;               
    ruido = desvio_padrao_ruido.*randn(3,1);
    valores_giroscopio = angular_velocities + ruido;
end

function valores_acelerometro = acelerometro(velocities,dt)
    global acceleration_auxiliar; 
    acelerations = ((velocities-acceleration_auxiliar)/dt) + [0;0;9.82]; 
    acceleration_auxiliar = velocities;
    desvio_padrao_ruido = 0;               
    ruido = desvio_padrao_ruido.*randn(3,1);
    valores_acelerometro = (acelerations + ruido);
end

function valores_barometro = barometer(altura)
    desvio_padrao_ruido = 0.1; 
    ruido = desvio_padrao_ruido*randn;
    valores_barometro = altura + ruido;
end

%construindo o magetronomo e gps utilizando o sistema de codenadas NED (North-East-Down)
% x aponta ao norte, y a este e z para baixo
function valores_magnetronomo = magnetonomer(angle) 
    rotZYX = eul2rotm(angle')'; %inercial para corpo
    desvio_padrao_ruido = 0; 
    ruido = desvio_padrao_ruido.*randn(3,1);
    valores_magnetronomo = rotZYX*[1;0;0] + ruido;
end

function valores_gps = gps(pos)
    desvio_padrao_ruido = 0.00001;               
    ruido = desvio_padrao_ruido.*randn(3,1);
    valores_gps = pos + ruido;
end

function y_next = rk4_step(ode_func, t, y, dt, method, varargin)
    if(method >1)
        k1 = ode_func(t, y, varargin{:});
        k2 = ode_func(t, y + dt*(1/5)*k1, varargin{:});
        k3 = ode_func(t, y + dt*(3/40)*k1 + dt*(9/40)*k2, varargin{:});
        k4 = ode_func(t, y + dt*(44/45)*k1 - dt*(56/15)*k2 + dt*(32/9)*k3, varargin{:});
        k5 = ode_func(t, y + dt*(19372/6561)*k1 - dt*(25360/2187)*k2 + dt*(64448/6561)*k3 - dt*(212/729)*k4, varargin{:});
        k6 = ode_func(t, y + dt*(9017/3168)*k1 - dt*(355/33)*k2 + dt*(46732/5247)*k3 + dt*(49/176)*k4 - dt*(5103/18656)*k5, varargin{:});
        y_next = y + dt * ((35/384)*k1 + 0*k2 + (500/1113)*k3 + (125/192)*k4 - (2187/6784)*k5 + (11/84)*k6);
    else 
        if(method>0)
            k1 = ode_func(t, y, varargin{:});
            k2 = ode_func(t + dt/2, y + (dt/2) * k1, varargin{:});
            k3 = ode_func(t + dt/2, y + (dt/2) * k2, varargin{:});
            k4 = ode_func(t + dt, y + dt * k3, varargin{:});
            y_next = y + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
        else
            k1 = ode_func(t, y, varargin{:});
            y_next = y + dt*k1;
        end
    end
end

function d_estados_dt = modelo_quad(t, estados, aeronave, control_action)
    % t - tempo
    % linear = [x;y;z;xdot;ydot;zdot];
    % zeta = [psi;theta;phi;psi_dot;theta_dot;phi_dot]
    % aeronave é uma struct com os parametros construtivos: quad = struct("Inertia", I_til, "mass", overall_mass, "Prop", Izz_motor);
    % Forças e torques gerados pelos motores, que temos o controle
    % Omega_res é a velocidade residual para conta do efeito giroscópio
    % Omega_res_dot será considerado 0, pois está se adotando que a
    % velocidade muda sem a dinâmica do motor.
    k_aero = aeronave.k_aero;
    k_drag = aeronave.q_aero;
    L = aeronave.Length;
    wh = aeronave.w_hover;
    for it=1:4
        if(control_action(it)>=(1.9*wh)^2)
            control_action(it) = (1.9*wh)^2;
        end
        if(control_action(it)<=(0.1*wh)^2)
            control_action(it) = (0.1*wh)^2;
        end
    end
    U = [-k_aero, -k_aero, -k_aero, -k_aero;
        0, -L*k_aero, 0, L*k_aero;
        L*k_aero, 0, -L*k_aero, 0;
        -k_drag, k_drag, -k_drag, k_drag]*control_action;
    omega_res = control_action(1)^0.5+control_action(3)^0.5-control_action(2)^0.5-control_action(4)^0.5;
    I_til = aeronave.Inertia;
    m = aeronave.mass;
    Jp = aeronave.Prop;
    g = aeronave.grav;

    Thrust = U(1);
    Torques = U(2:4);

    dx = estados(4);
    dy = estados(5);
    dz = estados(6);
    dpsi = estados(10);
    dtheta = estados(11);
    dphi = estados(12);
    rotZYX = eul2rotm(estados(7:9)');
    dLinear = (1/m)*([0;0;m*g]+rotZYX*[0;0;Thrust]);

    ddpsi = (1/I_til(9))*(Torques(3)+estados(11)*estados(12)*(I_til(1)-I_til(5)));
    ddtheta = (1/I_til(5))*(Torques(2)+estados(10)*estados(12)*(I_til(9)-I_til(1))-Jp*estados(12)*omega_res);
    ddphi = (1/I_til(1))*(Torques(1)+estados(11)*estados(10)*(I_til(5)-I_til(9))+Jp*estados(11)*omega_res);
   
    d_estados_dt = [dx;dy;dz; dLinear; dpsi; dtheta; dphi; ddpsi; ddtheta; ddphi];
end


