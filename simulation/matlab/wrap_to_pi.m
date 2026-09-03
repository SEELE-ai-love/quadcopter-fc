% wrap_to_pi.m
% Wraps an angle to (-pi, pi]. Vectorized.

function y = wrap_to_pi(x)
    y = mod(x + pi, 2*pi) - pi;
end
