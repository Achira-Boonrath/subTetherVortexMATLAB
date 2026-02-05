function R = perifocalToECI(Omega, i, omega)
%PERIFOCALTOECI Rotation from perifocal frame to ECI

cO = cos(Omega);  sO = sin(Omega);
ci = cos(i);      si = sin(i);
co = cos(omega);  so = sin(omega);

R = [ ...
    cO*co - sO*ci*so,   -cO*so - sO*ci*co,   sO*si;
    sO*co + cO*ci*so,   -sO*so + cO*ci*co,  -cO*si;
    si*so,               si*co,              ci
    ];

end