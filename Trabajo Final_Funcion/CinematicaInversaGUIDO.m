function [qOP,BOOL]=CinematicaInversaGUIDO
fprintf('######################################################\n')
fprintf('#              Cinematica Inversa GUIDO              #\n')
fprintf('######################################################\n\n')
%%Ingreso por teclado de los valores de T
T=zeros(4);T(4,4)=1;
let=[88;89;90];
flag="N";
while flag=="N"
    for i=1:3
        LET=char(let(i));
        fprintf('Ingrese orientacion de Efector Final en %s\n', LET);
        for j=1:3
            T(j,i)=input('');
        end
        fprintf('Ingrese posicion de Efector Final en %s\n',LET);
        T(i,4)=input('');    
    end
    fprintf('Ingrese vector de posiciones angulares (grados) anterior: \n');
    q_anterior=zeros(6,1);
    cont=0;
    for i=1:6
        cont=cont+1;
        q_anterior(cont,1)=input('');
    end
    fprintf('######################################################\n')
    fprintf('#######            DATOS DE ENTRADA            #######\n')
    fprintf('######################################################\n\n')
    fprintf('Matriz de Transformacion Homogenea:');
    T
    fprintf('Vector de Rotaciones anterior:');
    q_anterior
    q_anterior=q_anterior*pi/180;
    fprintf('\n----------------------------------------------------\n')
    fprintf('[Y] --> Si desea continuar con el calculo\n') 
    fprintf('[N] --> Si desea cambiar los datos\n')
    flag=input('[Y]/[N]:  ','s');
end
[qOP,BOOL]=CinematicaInversa_main(T,q_anterior);
BOOL
if BOOL == "FALSE"
    fprinf('La matriz de Transfrmacion ingresada es un punto fuera del espacio de trabajo\n')
    fprintf('El valor mas cercano dentro del espacio de trabajo en relacion al ingresado da un vector de rotaciones de: \n');
else
    fprintf('El punto esta dentro del espacio de trabajo y el q optimo segun su vector de posiciones articulares anterior es: \n');
fprintf('Vector de Rotaciones actual');
qOP
end

%T=[
%    0.8161    0.5327    0.2241   -0.6858
%    0.5667   -0.8137   -0.1294   -1.4389
%    0.1135    0.2326   -0.9659    0.6170
%         0         0         0    1.0000];
