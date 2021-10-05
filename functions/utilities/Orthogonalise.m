function OrthoMat = Orthogonalise(Mat)
%ORTHOGONALISE Summary of this function goes here
%   Detailed explanation goes here
check = 10;
while check > 1
    OrthoMat    = Mat + 0.5*(eye(size(Mat))-Mat*Mat')*Mat;
    check       = norm(OrthoMat)
    
end
