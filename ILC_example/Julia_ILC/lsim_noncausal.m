function output = lsim_noncausal(L, u, varargin)
    np = length(denpoly(L));
    nz = length(numpoly(L));
    zeroexcess = nz - np;
    if zeroexcess <= 0
        output = lsim(L, u, varargin{:});
        return;
    end
    
    integrators = tf(1, [1, 0], L.Ts)^zeroexcess;
    res = lsim(L * integrators, u, varargin{:});
    res.y(1:end-zeroexcess) = res.y(1+zeroexcess:end);
    output = res.y;
end
