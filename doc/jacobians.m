%% Calculates the Jacobians of the system

% state vector vars
syms x y vx vy psi

% input
syms ax ay oz

% time
syms T

% system model
g = [
    x + T*vx + (cos(oz*T + psi)*ax + sin(oz*T + psi)*ay) * T^2 / 2.0;
    y + T*vy + (sin(oz*T + psi)*ax + cos(oz*T + psi)*ay) * T^2 / 2.0;
    vx + (cos(oz*T + psi)*ax + sin(oz*T + psi)*ay) * T;
    vy + (sin(oz*T + psi)*ax + cos(oz*T + psi)*ay) * T;
    psi + oz*T;
    ];

% jacobians
G = jacobian(g, [x,y,vx,vy,psi]);
W = jacobian(g, [ax,ay,oz]);

% simplified jacobians
Gs = simplify(G);
Ws = simplify(W);

%% write to file (enables optimization)
comment = "Matlab generated code (check the docs) for symbolic expression: ";
ccode(Gs,'File','G.temp','Comments', comment + "Gs");
ccode(Ws,'File','W.temp','Comments', comment + "Ws");
