
% Takes in start and end time, acceleration, velocity and position
function coefficients = quinticPolynomial(ts, tf, qs, qf, vs, vf, as, af) 
    QPMatrix = [1 ts ts^2 ts^3 ts^4 ts^5;            %qs
                0 1 2*ts 3*(ts)^2 4*(ts)^3 5*(ts)^4; %vs
                0 0 2 6*ts 12*(ts)^2 20*(ts)^3;      %as
                1 tf tf^2 tf^3 tf^4 tf^5;            %qf
                0 1 2*tf 3*(tf)^2 4*(tf)^3 5*(tf)^4; %vf
                0 0 2 6*tf 12*(tf)^2 20*(tf)^3];     %af
    controlVector = [qs; vs; as; qf; vf; af];
    coefficients = QPMatrix \ controlVector;
    % coefficients = [a0 a1 a2 a3 a4 a5]
end