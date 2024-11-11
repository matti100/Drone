function y = measFunction(x,inputs)

u = inputs(1:4, 1);
mass = inputs(5,1);
g = inputs(6,1);

R_psi = @(x) [cos(x(9)),    -sin(x(9)),     0;
    sin(x(9)),     cos(x(9)),     0;
    0,             0,     1];

R_theta = @(x) [cos(x(8)),      0,      sin(x(8));
    0,      1,              0;
    -sin(x(8)),     0,       cos(x(8))];

R_phi = @(x) [1,        0,              0;
    0,        cos(x(7)),      -sin(x(7));
    0,        sin(x(7)),      cos(x(7))];

rotMat = @(x) R_psi(x) * R_theta(x) * R_phi(x);

acc = @(x, u) (1/mass).*rotMat(x)*[0; 0; u(1)] + [0; 0; -g];

y = [rotMat(x)' * (acc(x, u) + [0;0;-g]);
    x(10);
    x(11);
    x(12)
    ];

end

