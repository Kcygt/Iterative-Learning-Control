function ilc(Gc, Q, L,r,t)
    a = zeros(length(r)); % # ILC adjustment signal starts at 0
    for iter = 1:5
        ra = r + a;
        res = lsim(Gc, ra, t); % Simulate system, replaced by an actual experiment if running on real process
        y = res.y            ; % System response
        e = r - y           ; % Error
        Le = lsim_noncausal(L, e, t);
        a  = lsim_zerophase(Q, a + Le, t); % Update ILC adjustment
    end
    % plot(r)
    plot(res)
    plot(e)
end