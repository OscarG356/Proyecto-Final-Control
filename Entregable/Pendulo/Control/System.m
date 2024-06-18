clear all
close all
clc
%% Abrir data
load("dataPendulo.mat")

figure()
plot(U)
hold on
plot(Y)

SS.A(:,3) = 1e-4;
disp(SS.A)

A = SS.A;
B = SS.B;
C = SS.C(1,:);
D = SS.D(1);

SS = ss(A,B,C,D);

disp(A)
disp(B)
disp(C)
disp(D)

%% Control retroalimentación de estados - Continuo

Mp_des = 0.20;
ts_des = 8;
zita_des = sqrt(((log(Mp_des))^2)/(((log(Mp_des))^2)+pi^2));
wn_des = 4.6/(ts_des*zita_des);

polosCont = roots([1.,2*zita_des*wn_des,wn_des^2]);

rp = real(polosCont);
op1 = round(rp(1))*10;
op2 = op1-1;

polosCont = cat(1, polosCont,op1,op2);
disp(polosCont)

A_c = A;
B_c = B;

Kcont = place(A_c,B_c,polosCont);
disp(Kcont)

A_Cont = (A_c - B_c*Kcont);
B_Cont = B_c;

sysControlado = ss(A_Cont,B_Cont,C,D);


% Calcular la respuesta al escalón del sistema controlado
figure()
subplot(2,1,1)
step(sysControlado,'b');
legend;
grid on;
xlabel('Tiempo');
ylabel('Salida');
title('Salida con control');

subplot(2,1,2)
step(SS, 'r')
legend;
grid on;
xlabel('Tiempo');
ylabel('Salida');
title('Salida sin control');

%% Observador

rp = real(polosCont);
op1 = round(rp(1))*10;
op2 = op1-1;
op3 = op2-1;
op4 = op3-1;
Polos_obs = [op1,op2,op3,op4];
disp(Polos_obs)

A_ob = A';
C_ob = C';
L = place(A_ob,C_ob,Polos_obs);

L = L';
disp(L)

Acontrolado = A-L*C;
Bcontrolado = [B L];
Ccont = C;
Dcont = 0;
sysControlado_2 = ss(Acontrolado,Bcontrolado,Ccont,Dcont);


% Calcular la respuesta al escalón del sistema controlado
figure()
subplot(2,1,1)
step(sysControlado_2,'b');
legend;
grid on;
xlabel('Tiempo');
ylabel('Salida');
title('Salida con control');

subplot(2,1,2)
step(SS, 'r')
legend;
grid on;
xlabel('Tiempo');
ylabel('Salida');
title('Salida sin control');