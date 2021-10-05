function out = MakeStruc(varargin)
% MakeStruc accepts a variable number of inputs and constructs a single
% output struct from the inputs. The struct fields are named according to
% the name of the variables inputted.
% 
% Example:
%   >> a = 1; b=[2,3]; c=[0,0,0;1,1,1]; string='test string field';
%   >> MakeStruc(a, b, c, string)
% 
%  ans = 
%    
%   struct with fields:
% 
%          a: 1
%          b: [2 3]
%          c: [2x3 double]
%          string: 'test string field'

n   = length(varargin);
for i = 1:n
    name = inputname(i);
    out.(name) = varargin{i};
end

end