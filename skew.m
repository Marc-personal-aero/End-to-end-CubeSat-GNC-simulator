function S = skew(vec)

if length(vec) ~= 3
    error('Input vector must have 3 elements')
end

vec = vec(:);

S = [ 0, -vec(3), vec(2);
vec(3), 0, -vec(1);
-vec(2), vec(1), 0 ];

