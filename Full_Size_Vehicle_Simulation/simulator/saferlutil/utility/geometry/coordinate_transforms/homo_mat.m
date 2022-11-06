function [Homo] = homo_mat(theta,pos)
Rmat = rotmat(theta);
Homo = [Rmat reshape(pos,[2,1]); zeros(1,2) 1];
end

