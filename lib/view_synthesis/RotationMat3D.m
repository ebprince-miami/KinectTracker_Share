function R = RotationMat3D(a,b,g)

R = [cos(a)*cos(b) cos(a)*sin(b)*sin(g)-sin(a)*cos(g) cos(a)*sin(b)*cos(g)+sin(a)*sin(g);
    sin(a)*cos(b) sin(a)*sin(b)*sin(g)+cos(a)*cos(g) sin(a)*sin(b)*cos(g)-cos(a)*sin(g);
    -sin(b) cos(b)*sin(g) cos(b)*cos(g)];

