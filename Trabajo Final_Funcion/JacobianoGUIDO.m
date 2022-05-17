function [qGrados,detJpos]=JacobianoGUIDO
fprintf('######################################################\n')
fprintf('#                     JACOBIANO                      #\n')
fprintf('######################################################\n\n')
L1 = Link('d',0.575,'a',0.175,'alpha',pi/2);
L2 = Link('d',0,'a',0.890,'alpha',0,'offset',pi/4); %%pi/4
L3 = Link('d',0,'a',0,'alpha',pi/2,'offset',pi/4); %%pi/4
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

R = SerialLink([L1 L2 L3 L4 L5 L6],'name','GU1D0');
R.offset=[0 pi/4 pi/4 0 0 0];
j=0;
vecDET=zeros(1,63);

for i=1:1000
    Q(1,1)=randi([-185 185])*pi/180;
    Q(1,2)=randi([-105 130])*pi/180;
    Q(1,3)=randi([-165 120])*pi/180;
    Q(1,4)=randi([-180 180])*pi/180;
    Q(1,5)=randi([-125 125])*pi/180;
    Q(1,6)=randi([-350 350])*pi/180;
    J = R.jacob0(Q');
    detJ = det(J);
    matriz(1,i)=detJ;
    matriz(2:7,i)= Q*180/pi;
end

[minvalue,minindex] = min(abs(matriz(1,:)));
nums = 1:1:1000;
[B,I] = sort(abs(matriz(1,:)));
matrizOrdenada(1,:)=B;
for i=1:1000
    matrizOrdenada(2:7,i) = matriz(2:7,I(i));
end
matrizOrdenadaRAD(1:6,:) = matrizOrdenada(2:7,:)*pi/180;

%% Ploteo de las primeras 10 (mas cercanas a cero)

for i=1:10
    char = int2str(i);
    R(i) = SerialLink(dh,'name',char);
    figure (i)
    R(i).teach(matrizOrdenadaRAD(1:6,i)');   
end

figure (11)
hold on
scatter(matrizOrdenada(1,1:50),matrizOrdenada(7,1:50),'o');
scatter(matrizOrdenada(1,1:50),matrizOrdenada(6,1:50),'x');
ylabel('q');xlabel('det');

close all
%t = [0:0.02:2]';
%qq = jtraj(q1, q2, t);
%R.plot(qq);

%% Posición articular característica de la aplicación
qpos=[3.1416    0.3958   -1.4767    3.1416    0.4898   0];
qcero=zeros(1,6);
Rpos=SerialLink([L1 L2 L3 L4 L5 L6],'name','Colocacion o extraccion de rueda de automovil');
Jpos=Rpos.jacob0(qpos);
Jrpos=Jpos(1:3,1:3);
Tpos=Rpos.fkine(qpos);
C=Tpos.t;
%--------------------------------------------------------------------------

figure(12)
Rpos.teach(qpos);
%{
figure(13)
Rpos.teach('callback', @(r,qpos) r.vellipse(qpos))
hold on
plot_ellipse(Jrpos*Jrpos',C')
hold off
%}
%--------------------------------------------------------------------------
detJpos = det(Jpos);
fprintf('Determinante del Jacobiano en posicion elegida (Posici�n de la FIGURA 12) ')
detJpos
fprintf('Vector de variables articulares en grados')
qGrados=qpos*180/pi;
qGrados
end




