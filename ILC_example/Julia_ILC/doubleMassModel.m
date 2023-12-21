function G = doubleMassModel(Jl)

Jm = 1; 
k  = 100;
c0 = 1; 
c1 = 1;
c2 = 1;

A = [0 1 0 0;
    -k/Jm, -(c1 + c0)/Jm, k/Jm, c1/Jm;
    0 0 0 1;
    k/Jl c1/Jl -k/Jl -(c1 + c2)/Jl
    ];
B = [0; 1/Jm; 0; 0];
C = [1 0 0 0];
G = ss(A, B, C, 0);

