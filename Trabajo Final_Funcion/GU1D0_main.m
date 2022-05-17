function GU1D0_main
fprintf('######################################################\n')
fprintf('#                     ROBOT GUIDO                    #\n')
fprintf('######################################################\n\n')
%%Ingreso por teclado la opcion
flag = "Y";
while flag == 'Y'
    clear all,close all
    fprintf('1-Cinematica Directa\n2-Cinematica Inversa\n3-Jacobiano \n4-Trayectoria\n\n')
    valor=input('Ingrese lo que desea ejecutar:\n') ;
    switch valor
        case 1
            [~,~,~]=CinematicaDirectaGUIDO;
        case 2
            [~,~]=CinematicaInversaGUIDO;
        case 3
            fprintf("Devuelve 10 figuras con el robot en posiciones aleatorias, con las cuales se analizaron singularidades \n");
            [~,~]=JacobianoGUIDO;
        case 4
            [~]=TrayectoriaGUIDO;
    end
    fprintf('\n----------------------------------------------------\n')
    fprintf('[Y] --> Si desea continuar\n') 
    fprintf('[N] --> Si desea salir\n')
    flag=input('[Y]/[N]:  ','s');
    if flag=="N"
        fprintf('ADIOS\n')
    end
end
end