function height = GetHeadHeightWrtFloor(Pos3D, R, t)

p = R*Pos3D'+repmat(t,[1 size(Pos3D,1)]);

height = -p(3,:);