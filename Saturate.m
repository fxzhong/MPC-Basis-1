function a = Saturate(x, sat)

if x > sat
    a = sat;
elseif x < -sat
    a = -sat;
else
    a = x;
end