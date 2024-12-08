function [LSE,D] = ToA(meas_dn)
persistent firstRun
persistent H_pseudoInv d1 d2

if isempty(firstRun)
    firstRun = 1;
    d1=10;
    d2=d1;
    H = [...
        0, -2*d2
        2*d1, -2*d2
        2*d1, 0
        2*d1, 0
        2*d1, 2*d2
        0, 2*d2];
    H_pseudoInv = (H'*H)\H';
end

D = [...
    meas_dn(1,1)^2 - meas_dn(2,1)^2 - d2^2
    meas_dn(1,1)^2 - meas_dn(3,1)^2 + d1^2 - d2^2
    meas_dn(1,1)^2 - meas_dn(4,1)^2 + d1^2
    meas_dn(2,1)^2 - meas_dn(3,1)^2 + d1^2
    meas_dn(2,1)^2 - meas_dn(4,1)^2 + d1^2 + d2^2
    meas_dn(3,1)^2 - meas_dn(4,1)^2 + d2^2
    ];

LSE = H_pseudoInv * D;

end