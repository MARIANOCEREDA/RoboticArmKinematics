function [qOP,BOOL,ningunoCumple]=CinematicaInversa(T,q_anterior,vecqlimGRAD,dh,R)
R.offset=[0 pi/4 pi/4 0 0 0];
R.tool=eye(4);
R.tool=transl(0,0,0.3);
R.base = eye(4);
%q_anterior = [0 0 0 0 0 0];
%% ---------------CALCULO DE POSICION----------------- %%
T = invHomog(R.base.double)*T*invHomog(R.tool.double);
p04=T(1:3,4)- dh(6,2)* T(1:3,3);  %%T(1:3,3) contiene el versor z.

%% DETERMINACION DE q1 - son 2 valores
qtot=zeros(6,8);
q1(1)=atan2(p04(2),p04(1)); %%tg-1(py/pz)
%Tenemos 2 resultados de q1, para llegar a un mismo lugar en el espacio:
if (q1(1)>0)
    q1(2)=q1(1)-pi;
else
    q1(2)=q1(1)+pi;
end
qtot(1,1:4)=q1(1);
qtot(1,5:8)=q1(2);
%% DETERMINACION DE q2 - tenemos 2 valores para cada valor de q1
j=0;
q2=zeros(1,4);
for i=1:2
    T1 = trotz(q1(i))*transl(0,0,dh(1,2))*transl(dh(1,3), 0, 0)*trotx(dh(1,4)); %T1 es T1 respecto de 0
    p14 = invHomog(T1)*[p04;1]; %%Obtenemos el punto 4 respecto de 1
    beta = atan2(p14(2),p14(1));
    L2 = dh(2,3); %Longitud eslabon 2
    L3 = dh(4,2); %Longitud eslabon 3
    diag = sqrt(p14(1)^2 + p14(2)^2);
    alfa = acos((L2^2-L3^2+diag^2)/(2*diag*L2)); %Teorema del coseno
    verificacionAlfa(1,i) = isreal(alfa);
    j=j+1;
    q2(j)=beta-real(alfa); %Obtenemos 2 valores de q2 para cada valor de q17
    j=j+1;
    q2(j)=beta+real(alfa);
end
qtot(2,:)=[q2(1) q2(1) q2(2) q2(2) q2(3) q2(3) q2(4) q2(4)];
%% DETERMINACION DE q3 - Tenemos 1 valor por cada valor de q2
% ya conocemos T10
j=1;
q3=zeros(1,4);
 for i=1:2
     T1 = trotz(q1(1))*transl(0,0,dh(1,2))*transl(dh(1,3), 0, 0)*trotx(dh(1,4));
     T12 = trotz(q2(i))*transl(0,0,dh(2,2))*transl(dh(2,3), 0, 0)*trotx(dh(2,4)); %T2 respecto de 0
     T2 = T1*T12; %%Obtengo T de 2 respecto de 0
     p24 = invHomog(T2)*[p04;1]; 
     q3(j)= atan2(p24(2),p24(1))+pi/2;
     j=j+1;
 end
j=3;
for i=3:4
     T1 = trotz(q1(2))*transl(0,0,dh(1,2))*transl(dh(1,3), 0, 0)*trotx(dh(1,4));
     T12 = trotz(q2(i))*transl(0,0,dh(2,2))*transl(dh(2,3), 0, 0)*trotx(dh(2,4)); %T2 respecto de 0
     T2 = T1*T12; %%Obtengo T de 2 respecto de 0
     p24 = invHomog(T2)*[p04;1]; 
     q3(j)= atan2(p24(2),p24(1))+pi/2;
     j=j+1;
end

qtot(3,:)= [q3(1) q3(1) q3(2) q3(2) q3(3) q3(3) q3(4) q3(4)];
%% ----------------CALCULO DE ORIENTACION------------------ %%
k=1;
for i=1:2:8
    T10 = A_dh(dh(1,:),qtot(1,i));
    T21 = A_dh(dh(2,:),qtot(2,i));
    T32 = A_dh(dh(3,:),qtot(3,i));
    T63 = invHomog(T32)*invHomog(T21)*invHomog(T10)*T;
    if abs(T63(3,3) - 1) < eps
        % solución degenerada:
        %   > z3 y z5 alineados (y z6)
        %   > q4 y q6 generan el mismo movimiento
        %   > q5 = 0 (o q5 = 180º)
        %   > se asume q4 = q4_anterior
        q4(1) = q_anterior(4);
        q5(1) = 0;
        q6(1) = atan2(T63(2,1), T63(1,1)) - q4(1);
        q4(2) = q4(1);
        q5(2) = 0;
        q6(2) = q6(1);
    else
        % solución normal:
        q4(1) = atan2(-T63(2,3), -T63(1,3));
        if (q4(1)>0)
            q4(2) = q4(1) - pi; 
        else
            q4(2) = q4(1) + pi;
        end
        q5 = zeros(1,2);
        q6 = q5;
        for m=1:2
            T43 = A_dh(dh(4,:), q4(m));
            T46 = invHomog(T43)*T63;
            q5(m) = atan2(-T46(1,3), T46(2,3))+pi;
            T45 = (trotz(q5(m))*transl(0,0,dh(5,2))*transl(dh(5,3), 0, 0)*trotx(dh(5,4)));
            T56 = invHomog(T45)*T46;
            q6(m) = atan2(T56(2,1), T56(1,1));
        end
    end
qtot(4,k:k+1)=q4(1:2); 
qtot(5,k:k+1)=q5(1:2); 
qtot(6,k:k+1)=q6(1:2);
k = k+2;
end
qtot = qtot - R.offset' * ones(1,8);
qgrad=qtot*180/pi;
q_anteriorGrad=q_anterior;
qt = qtot(:,6)';

%% VERIFICACION MATEMATICA DE QUE EL PUNTO ESTE DENTRO DEL ESPACIO DE TRABAJO
if verificacionAlfa(1)==1
    BOOL = "TRUE";
else 
    BOOL = "FALSE";
end

%% CALCULO DE LA SOLUCION MAS CERCANA A LA POSICION ANTERIOR y  VERIFICACION DE CUMPLIMIENTO CON LIMITES ARTICULARES
condicion=1;
contador = 0;
n=6;
verifLimites = zeros(1,n);
qOP=zeros(6,1);
ningunoCumple = 0;
moduloDiferencia = zeros(1,6);
for j=1:8
    sum=0;
    for i=1:6
        diferencia(i,j) = abs(qgrad(i,j) - q_anteriorGrad(i));
        sum = sum + diferencia(i,j)^2;
    end
    moduloDiferencia(j) = sqrt(sum);
end

while condicion==1
    [value,index] = min(moduloDiferencia);
    qOP = qgrad(:,index);
    for i=1:n
        if(qOP(i)<vecqlimGRAD(i,1) || qOP(i)>vecqlimGRAD(i,2))
           verifLimites(i) = 0; %0=FALSE
        else
           verifLimites(i)= 1; %1=TRUE
        end
    end
    if(min(verifLimites)==1)
        condicion=0;
    else
        moduloDiferencia(index)=max(moduloDiferencia);
    end
    if contador == 8
        ningunoCumple = 1;
        condicion = 0;
    end
    contador=contador + 1;
end
end

function iT = invHomog(T)
iT = eye(4);
iT(1:3,1:3) = T(1:3,1:3)';
iT(1:3,4) = - iT(1:3,1:3) * T(1:3,4);
end

function dh2T = A_dh(dh,q)
dh2T = trotz(q)*transl(0,0, dh(1,2))*transl(dh(1,3), 0, 0)*trotx(dh(1,4));
end



