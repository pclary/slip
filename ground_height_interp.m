function y = ground_height_interp(x, ground_data)

y = zeros(size(x));

for i = 1:length(x)
    [~, yi] = linexpoly([x(i) x(i)], [-1e3 1e3], ground_data(:, 1), ground_data(:, 2));
    if ~isempty(yi)
        y(i) = max(yi);
    else
        y(i) = NaN;
    end
end


function [xi, yi, ii] = linexpoly(x1, y1, x2, y2)
% Customized implementation of polyxpoly for codegen
dx1 = x1(2) - x1(1);
dy1 = y1(2) - y1(1);
dx2 = x2(2:end) - x2(1:end-1);
dy2 = y2(2:end) - y2(1:end-1);
dx12 = x1(1) - x2(1:end-1);
dy12 = y1(1) - y2(1:end-1);

num = dx2.*dy12 - dy2.*dx12;
den = dy2.*dx1 - dx2.*dy1;

sa = num./den;
sb = (dx12 + sa.*dx1)./dx2;

ii = find(den ~= 0 & sa >= 0 & sa < 1 & sb >= 0 & sb < 1);
xi = x1(1) + sa(ii).*dx1;
yi = y1(1) + sa(ii).*dy1;
