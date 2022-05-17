function [RE,qCorregido,CinDirec]=verificacionLimites(q1,q2,q3,q4,q5,q6)
%Funcion para ingresar valores de q segun parametros DH
%
%Pide 6 valores para q1, q2, q3, q4, q5 y q6 respectivamente
%Los valores a ingresar son angulos en GRADOS
%
%El retorno depende de los valores ingresados, si estan dentro de los
%limites, entonces devuelve un true (0), de lo contrario un false (1). Ademas, de
%ser false, el programa indica la posicion permitida mas cercana (limite)
%
%[retorno,qCorregido]= verificacionLimites(q1,q2,q3,q4,q5,q6)

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

%%Vector de variables q de entrada
q=[q1,q2,q3,q4,q5,q6];

n=6;
vecqlim = round([L1.qlim;L2.qlim;L3.qlim;L4.qlim;L5.qlim;L6.qlim]*180/pi);
retorno = zeros(n,6);
qCorregido = zeros(1,n);
for i=1:n
    if(q(i)<vecqlim(i,1) || q(i)>vecqlim(i,2))
       retorno(i,:) = [70 65 76 83 69 32];
       if (q(i)<0)
           qCorregido(i)= vecqlim(i,1);
       elseif (q(i)>0)
           qCorregido(i)= vecqlim(i,2);    
       end
    else
        qCorregido(i)=q(i);
        retorno(i,:)= [84 82 85 69 32 32];
    end
end
i=1;
for j=6:6:36
    retornoFila(1,j-5:j)=retorno(i,:);
    i=i+1;
end
%%RE=char(retorno);
RE=char(retornoFila);
qPLOT=qCorregido*(pi/180);
robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'GU1D0');
robot.tool=eye(4);
robot.tool=transl(0,0,0.3);
robot.base = eye(4);
robot.teach(qPLOT)
CinDirec=robot.fkine(qPLOT).double;
%%robot.tool = transl(0,0,0.3);
%%robot.base = transl(0,-1,0);
%%robot.fkine(q)
end
