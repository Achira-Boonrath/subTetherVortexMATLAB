function y = counter()
    persistent k
    if isempty(k)
        k = 0;
    end
    k = k + 1;
    y = k;
end