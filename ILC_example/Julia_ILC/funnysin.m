function r = funnysin(x)
    x = sin(x);
    r = sign(x)*((abs(x) + 0.01)^0.2 - 0.01^0.2);
end