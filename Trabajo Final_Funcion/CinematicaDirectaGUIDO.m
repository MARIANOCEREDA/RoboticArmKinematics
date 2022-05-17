function [retornoVL,qCorregido,CinDirec]=CinematicaDirectaGUIDO
fprintf('######################################################\n')
fprintf('#                 CINEMATICA DIRECTA                 #\n')
fprintf('######################################################\n\n')
for i=1:6
    q(i)=input('Ingrese los 6 valores de q( en grados): \n');
end
fprintf('False -> valor fuera de rango \n');
fprintf('True -> valor dentro del rango \n');
[retornoVL,qCorregido,CinDirec] = verificacionLimites(q(1),q(2),q(3),q(4),q(5),q(6));
fprintf('Vector de q ingresado: \n');
disp(q);
fprintf('Valores correctos e incorrectos ingresados: \n');
disp(retornoVL);
fprintf('\nVector de q corregido: \n');
disp(qCorregido);
fprintf('\nMatriz de Transformacion Directa: \n');
disp(CinDirec);
end