function out = NormDB(data)

out.normvals        = zeros(size(data,2),2);

for col = 1:size(data,2)
    
    minv        = min(data(:,col));
    maxv        = max(data(:,col));
    
    if minv == maxv
        minv = 0;
    else
    end
    
    data(:,col) = (data(:,col)-minv)./(maxv-minv);
%     data(:,col) = (data(:,col)-minv)./diff;
    
    
    out.normvals(col,:) = [minv,maxv];
    
end

[X,Y,Z,CLV]     = Format3Dtab(data,4);
[~,~,~,CDV]     = Format3Dtab(data,5);
[~,~,~,CmV]     = Format3Dtab(data,6);

[Xq,Yq,Zq]      = meshgrid(0:0.1:1,0:0.1:1,0:0.1:1);
CLs             = interp3(X,Y,Z,CLV,Xq,Yq,Zq,'linear');
CDs             = interp3(X,Y,Z,CDV,Xq,Yq,Zq,'linear');
Cms             = interp3(X,Y,Z,CmV,Xq,Yq,Zq,'linear');

out.Xq  = Xq;
out.Yq  = Yq;
out.Zq  = Zq;
out.CLs = CLs;
out.CDs = CDs;
out.Cms = Cms;

end