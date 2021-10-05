function [X,Y,Z,V] = Format3Dtab(data,index)
% Input: data in format as exported from aerodb case, column of values
%           input takes '2d' table and converts into meshgrid
% Output: X,Y,Z - meshgridded variables for aoa, Mach and deflection
%         V - array of values in required format for interp3
% Example call: interp3(X,Y,Z,permute(V,[2 1 3]),2,1,0)


c1 = unique(data(:,1));
c2 = unique(data(:,2));
c3 = unique(data(:,3));

[X,Y,Z] = meshgrid(c1,c2,c3);

a = length(c1)*length(c2);
b = length(c1);

V = zeros(length(c1),length(c2),length(c3));
for k = 1:length(c3)
    for j = 1:length(c2)
        for i = 1:length(c1)
            V(i,j,k) = data((k-1)*a+(j-1)*b+i,index);
        end
    end
end

V = permute(V,[2 1 3]);

end
