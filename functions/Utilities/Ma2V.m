function v = Ma2V(Ma, h)

    [T,~,~] = GetAtmo(h);
    sos     = sqrt(1.4*287.053*T);
    v       = Ma*sos;

end