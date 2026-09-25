function R = Body2ECI_quat(q0123)
%%% Vector(ECI) = Body2ECI_quat * Vector(body)

q0 = q0123(1);
q1 = q0123(2);
q2 = q0123(3);
q3 = q0123(4);

q0sq = q0^2;
q1sq = q1^2;
q2sq = q2^2;
q3sq = q3^2;

R = [(q0sq+q1sq-q2sq-q3sq),    2*(q1*q2-q0*q3),       2*(q0*q2+q1*q3);
    2*(q1*q2+q0*q3),        (q0sq-q1sq+q2sq-q3sq),    2*(q2*q3-q0*q1);
    2*(q1*q3-q0*q2),          2*(q0*q1+q2*q3),     (q0sq-q1sq-q2sq+q3sq)];
