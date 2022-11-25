X = [1;1;1];
X_wp = [5;9;2.5];
X_goal = [9;1;4];
tf1 = 30;tf2=30;
coeff = zeros(6,6);
for i = 1:3
    a = X(i);
    b= X_wp(i);
    t = tf1;
    coeff(:,i)=polycoeff(a,b,t);
end
for i = 1:3
    a = X_wp(i);
    b= X_goal(i);
    t = tf2;
    coeff(:,i+3)=polycoeff(a,b,t);
end

function coeff = polycoeff(a,b,tf)
    lhs=[a,b,0,0,0,0]';
    mat = [0,0,0,0,0,1;tf^5,tf^4,tf^3,tf^2,tf^1,1;0,0,0,0,1,0;5*tf^4,4*tf^3,3*tf^2,2*tf^1,1,0;0,0,0,2,0,0;20*tf^3,12*tf^2,6*tf,2,0,0];
    coeff = mat\lhs;
end