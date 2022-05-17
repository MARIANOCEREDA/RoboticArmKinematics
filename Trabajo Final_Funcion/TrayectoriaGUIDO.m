function [QTOTAL]=TrayectoriaGUIDO

fprintf('######################################################\n')
fprintf('#                    TRAYECTORIA                     #\n')
fprintf('######################################################\n\n')
%% Datos del ROBOT
L1 = Link('d',0.575,'a',0.175,'alpha',pi/2);
L2 = Link('d',0,'a',0.890,'alpha',0); %%pi/4
L3 = Link('d',0,'a',0,'alpha',pi/2); %%pi/4
L4 = Link('d',1.035,'a',0,'alpha',-pi/2); %
L5 = Link('d',0,'a',0,'alpha',pi/2); 
L6 = Link('d',0.185,'a',0,'alpha',0); 
 
L1.qlim(1,1:2)=[-185,185]*pi/180;
L2.qlim(1,1:2)=[-105,130]*pi/180;
L3.qlim(1,1:2)=[-165,120]*pi/180;
L4.qlim(1,1:2)=[-180,180]*pi/180;
L5.qlim(1,1:2)=[-125,125]*pi/180;
L6.qlim(1,1:2)=[-350,350]*pi/180;

vecqlimGRAD = round([L1.qlim;L2.qlim;L3.qlim;L4.qlim;L5.qlim;L6.qlim]*180/pi);

dh = [0 0.575 0.175 pi/2;
      0 0 0.89 0;
      0 0 0 pi/2;
      0 1.035 0 -pi/2;
      0 0 0 pi/2;
      0 0.185 0 0];

R1 = SerialLink([L1 L2 L3 L4 L5 L6],'name','GU1D01 - SOLO JTRAJ');

R1.offset=[0 pi/4 pi/4 0 0 0];
R1.base=eye(4);
R1.tool=transl(0,0,0.3);

%% Puntos y coordenadas a interpolar
p1 = [-1;0;0];
q1 = [3.1416    0.3958   -1.4767    3.1416    0.4898   0];
p2 = [-1;0;1];
q2 = [ 3.1416    1.0225   -0.9791    3.1416    1.6142   0];
p3 = [-1.8;1.2;0.5];
q3 = [2.7635   -0.3809   -0.0424    1.4090    1.2275    0.4515];
p4 = [-1.8;1.5;0.5];
q4 = [2.6281   -0.6278    0.4188   1.4544    1.0696    0.2388];


%% -----------PARTE 3-a (JTRAJ)-------------
[Q1,QD1,QDD1] = jtraj(q1,q2,100);
[Q2,QD2,QDD2] = jtraj(q2,q3,100);
[Q3,QD3,QDD3] = jtraj(q3,q4,100);
Q = [Q1;Q2;Q3];

figure
R1.teach(Q,'delay',0.001,'trail', {'b'});

% Posicion
figure
subplot(3,1,1);qplot(Q1);title("Posici�n - Primer trayecto (JTRAJ)"); % gráficas
subplot(3,1,2);qplot(Q2);title("Posici�n - Segundo trayecto (JTRAJ)"); % gráficas
subplot(3,1,3);qplot(Q3);title("Posici�n - Tercer trayecto (JTRAJ)"); % gráficas

% Velocidad
figure
subplot(3,1,1);qplot(QD1);title("Velocidad - Primer trayecto (JTRAJ)"); % gráficas
subplot(3,1,2);qplot(QD2);title("Velocidad - Segundo trayecto (JTRAJ)"); % gráficas
subplot(3,1,3);qplot(QD3);title("Velocidad - Tercer trayecto (JTRAJ)"); % gráficas

%Aceleración
figure
subplot(3,1,1);qplot(QDD1);title("Aceleraci�n - Primer trayecto (JTRAJ)"); % gráficas
subplot(3,1,2);qplot(QDD2);title("Aceleraci�n - Segundo trayecto (JTRAJ)"); % gráficas
subplot(3,1,3);qplot(QDD3);title("Aceleraci�n - Tercer trayecto (JTRAJ)"); % gráficas
%}
%% -----------PARTE 3-b (CTRAJ)-----------------
R2 = SerialLink([L1 L2 L3 L4 L5 L6],'name','GU1D02 - SOLO CTRAJ');
R2.offset=[0 pi/4 pi/4 0 0 0];
R2.base=eye(4);
R2.tool=transl(0,0,0.3);

T1 = [1 0 0 -1;
      0 -1 0 0;
      0 0 -1 0;
      0 0 0 1];
T2 = [1 0 0 -1;
      0 -1 0 0;
      0 0 -1 1;
      0 0 0 1];
T3 = [-1 0 0 -1;0 1 0 0;0 0 -1 1;0 0 0 1];
T4 = [1 0 0 -1.8;0 0 1 1.2;0 -1 0 0.5;0 0 0 1];
T5 = [1 0 0 -1.8;0 0 1 1.5;0 -1 0 0.5;0 0 0 1];
TT1 = ctraj(T1,T2,100);
TT2 = ctraj(T2,T3,100);
TT3 = ctraj(T3,T4,100);
TT4 = ctraj(T4,T5,100);
q_anterior = zeros(400,6);
for i=1:100
    [qOP,BOOL,ningunoCumple]=CinematicaInversa(TT1(:,:,i),q_anterior(i,:),vecqlimGRAD,dh,R2);
    nc(i)=ningunoCumple;
    qTOT(i,:)=qOP;
    q_anterior(i,:) = qOP;
end
for i=1:100
    j=i+100;
    [qOP,BOOL,ningunoCumple]=CinematicaInversa(TT2(:,:,i),q_anterior(j-1,:),vecqlimGRAD,dh,R2);
    nc(j)=ningunoCumple;
    qTOT(j,:)=qOP;
    q_anterior(j,:) = qOP;
end
for i=1:100
    k=i+200;
    [qOP,BOOL,ningunoCumple]=CinematicaInversa(TT3(:,:,i),q_anterior(k-1,:),vecqlimGRAD,dh,R2);
    nc(k)=ningunoCumple;
    qTOT(k,:)=qOP;
    q_anterior(k,:) = qOP;
end
for i=1:100
    l=i+300;
    [qOP,BOOL,ningunoCumple]=CinematicaInversa(TT4(:,:,i),q_anterior(l-1,:),vecqlimGRAD,dh,R2);
    nc(l)=ningunoCumple;
    qTOT(l,:)=qOP;
    q_anterior(l,:) = qOP;
end
Qctraj = qTOT*pi/180;
Q1c=Qctraj(1:100,:);
Q2c=Qctraj(100:200,:);
Q3c=Qctraj(200:300,:);
Q4c=Qctraj(300:400,:);

figure
R2.teach(Qctraj,'delay',0.001,'trail', {'b'}); % animación

figure
subplot(4,1,1);qplot(Q1c);title("Posicion - Primer trayecto (CTRAJ)"); 
subplot(4,1,2);qplot(Q2c);title("Posicion - Segundo trayecto (CTRAJ)"); 
subplot(4,1,3);qplot(Q3c);title("Posicion - Tercer trayecto (CTRAJ)");
subplot(4,1,4);qplot(Q4c);title("Posicion - Cuarto trayecto (CTRAJ)");

QD1c=diff(Q1c)*100;
QD2c=diff(Q2c)*100;
QD3c=diff(Q3c)*100;
QD4c=diff(Q4c)*100;

figure
subplot(4,1,1);qplot(QD1c);title("Velocidad - Primer trayecto (CTRAJ)");
subplot(4,1,2);qplot(QD2c);title("Velocidad - Segundo trayecto (CTRAJ)");
subplot(4,1,3);qplot(QD3c);title("Velocidad - Tercer trayecto (CTRAJ)");
subplot(4,1,4);qplot(QD4c);title("Velocidad - Cuarto trayecto (CTRAJ)");

QDD1c=diff(QD1c)*100;
QDD2c=diff(QD2c)*100;
QDD3c=diff(QD3c)*100;
QDD4c=diff(QD4c)*100;

figure
subplot(4,1,1);qplot(QDD1c);title("Aceleracion - Primer trayecto (CTRAJ)");
subplot(4,1,2);qplot(QDD2c);title("Aceleracion - Segundo trayecto (CTRAJ)");
subplot(4,1,3);qplot(QDD3c);title("Aceleracion - Tercer trayecto (CTRAJ)");
subplot(4,1,4);qplot(QDD4c);title("Aceleracion - Cuarto trayecto (CTRAJ)");

%% MEZCLA DE TRAYECTORIAS - TRAYECTO FINAL

R3 = SerialLink([L1 L2 L3 L4 L5 L6],'name','GU1D03 - JTRAJ Y CTRAJ COMBINADOS');
R3.offset=[0 pi/4 pi/4 0 0 0];
R3.base=eye(4);
R3.tool=transl(0,0,0.3);

figure
qplot(Qctraj*180/pi); % gráficas
QTOTAL = [Q1c;Q2;Q4c];
R3.teach(QTOTAL,'delay',0.001,'trail', {'b'}); % animación

%-------------------------------------------------------------------------
% Posicion
figure
subplot(3,1,1);qplot(Q1);title("Posicion - Primer trayecto (CTRAY Y JTRAJ)");  % gráficas
subplot(3,1,2);qplot(Q2c);title("Posicion - Segundo trayecto (CTRAY Y JTRAJ)");  % gráficas
subplot(3,1,3);qplot(Q4c);title("Posicion - Tercer trayecto (CTRAY Y JTRAJ)");  % gráficas

% Velocidad
figure
subplot(3,1,1);qplot(QD1c);title("Velocidad - Primer trayecto (CTRAY Y JTRAJ)");  % gráficas
subplot(3,1,2);qplot(QD2);title("Velocidad - Segundo trayecto (CTRAY Y JTRAJ)");  % gráficas
subplot(3,1,3);qplot(QD4c);title("Velocidad - Tercer trayecto (CTRAY Y JTRAJ)");  % gráficas

%Aceleración
figure
subplot(3,1,1);qplot(QDD1c);title("Aceleracion - Primer trayecto (CTRAY Y JTRAJ)");  % gráficas
subplot(3,1,2);qplot(QDD2);title("Aceleracion - Segundo trayecto (CTRAY Y JTRAJ)");  % gráficas
subplot(3,1,3);qplot(QDD4c);title("Aceleracion - Tercer trayecto (CTRAY Y JTRAJ)");  % gráficas
