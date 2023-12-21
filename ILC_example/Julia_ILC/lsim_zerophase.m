function output = lsim_zerophase(G, u, varargin)
    res = lsim(G, u(:, end:-1:1), varargin{:});
    output = lsim(G, res.y(:, end:-1:1), varargin{:});
    output = output.y;
end

