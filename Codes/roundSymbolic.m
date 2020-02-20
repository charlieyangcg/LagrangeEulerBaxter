function [ simpEle ] = roundSymbolic(mEle, deg)
% roundSymbolic Round coefficients in symbolic expression
%   Takes a symbolic expression, breaks it down into coefficients and 
%   terms, rounds to zero if less than specified degree. 
[cof, trm] = coeffs(mEle);
fprintf('There are %d coeffs to round: \n', length(cof)) 
parfor i = 1:length(cof)
    disp(i)
    cof(i) = round(cof(i)*10^deg)/10^deg;
end
simpEle = dot(cof,trm);
end

