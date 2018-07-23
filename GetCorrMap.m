function c = GetCorrMap(Im, template)

if mod(size(template,1), 2) == 0 || mod(size(template,2), 2) == 0
    disp('template width and height must be odd number');
    return;
end
offx = uint16(size(template, 2) / 2);
offy = uint16(size(template, 1) / 2);
try
    c = normxcorr2(template, Im);
    c = c(offy+1:offy+size(Im,1), offx+1:offx+size(Im,2)); 
catch
    c = 1;
end
end