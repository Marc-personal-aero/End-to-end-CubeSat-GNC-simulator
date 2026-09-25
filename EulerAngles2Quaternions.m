function q = EulerAngles2Quaternions(EulerAng)
phi = EulerAng(1);
theta = EulerAng(2);
psi = EulerAng(3);

q0 = cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2);
q1 = sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2);
q2 = cos(phi/2)*sin(theta/2)*cos(psi/2) + sin(phi/2)*cos(theta/2)*sin(psi/2);
q3 = cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*sin(theta/2)*cos(psi/2);

q = [q0;q1;q2;q3];