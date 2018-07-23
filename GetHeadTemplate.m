function [Im_t, Id_t, bbox] = GetHeadTemplate(Im_e, Id_e, Im)

if nargin >2
    figure;imshow(Im);
else
    figure;imshow(Im_e);
end
[x, y] = ginput(2);
x = floor(x / 2) * 2;
y = floor(y / 2) * 2;

Im_t = Im_e(y(1):y(2),x(1):x(2));
Id_t = Id_e(y(1):y(2),x(1):x(2));

bbox = [x(1) y(1) x(2)-x(1) y(2)-y(1)];

end