function dh2T = A_dh(dh,q)
dh2T = trotz(q)*transl(0,0, dh(1,2))*transl(dh(1,3), 0, 0)*trotx(dh(1,4));
end