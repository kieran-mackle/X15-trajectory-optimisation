function K = GetK(e,K)

if K == 1e-10
    return
else
    orders      = -5:1:8;
    Ks          = linspace(1e-10,K,length(orders));

    o           = floor(log(abs(e))./log(10));

    K           = interp1(orders,Ks,o);
end
end
