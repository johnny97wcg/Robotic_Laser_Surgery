%returns the angle  equation at a specific time t
function q = get_instant_angle(a, t) 
% a is the vector of coefficients from quinticPolynomial
  q = a(1) + a(2)*t + a(3)*t^2 + a(4)*t^3 + a(5)*t^4 + a(6)*t^5;
end
