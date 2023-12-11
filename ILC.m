function [ ] = ILC(sysd,Ts)

    % Get state space values for ILC
    [Ad Bd Cd Dd] = ssdata(sysd);

    % Initial condition x0, time range t - assume 1 second, pure time delay n0, relative
    % degree r, and matrix size N
    x0 = 0;
    t = 0:Ts:1;
    n0 = 0;
    r = 1;
    N = length(t);

    % Define input vector U and reference J - Refernce = input for this example
    Rj = [5*ones(1,20) 2*ones(1,20) 4*ones(1,20) 5*ones(1,20) 0*ones(1,21)]';
    U = Rj;

    % G0 not formulated as initial condition is 0

    % Formulate G
    Gvec = zeros(N,1);
    rVec = ((r-1):(N-n0-1))';
    
    for ii = 1:length(rVec)
      ApowVec = Ad^rVec(ii);
      Gvec(ii) = Cd*ApowVec*Bd;
    end

    G = tril(toeplitz(Gvec));

    % Set up ILC
    jmax = 15;
    
    l0 = 0.95; L = l0 * eye(N,N);
    q0 = 1.00; Q = q0 * eye(N,N);
    
    Uj = zeros(N,1); Ujold = Uj;
    Ej = zeros(N,1); Ejold = Ej;
    
    % Run ILC and plot the response for each iteration
    for ii = 1:jmax
      Uj = Q*Ujold + L*Ejold;
      Yj = G*Uj;

      Ej = Rj - Yj; Ej(1) = 0;

      Ejold = Ej;
      Ujold = Uj;

      plotter(ii,t,Ej,Yj,Uj,Rj,U)
    end

end