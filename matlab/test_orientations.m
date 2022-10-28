syms a b d e D_x D_y W_x W_y

res = solve([D_x == a*W_x + b*W_y, D_y == d*W_x + e*W_y],[W_x, W_y]);




%%

eul = [0.006625, 0.016674, -0.279253];

cy = cos(eul(3) * 0.5);
sy = sin(eul(3) * 0.5);
cp = cos(eul(2) * 0.5);
sp = sin(eul(2) * 0.5);
cr = cos(eul(1) * 0.5);
sr = sin(eul(1) * 0.5);

q = zeros(4,1);
q(1) = cr * cp * cy + sr * sp * sy;
q(2) = sr * cp * cy - cr * sp * sy;
q(3) = cr * sp * cy + sr * cp * sy;
q(4) = cr * cp * sy - sr * sp * cy;