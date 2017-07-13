function [proj_points, t, R] = ar_cube(H,render_points,K)
%% ar_cube
% Estimate your position and orientation with respect to a set of 4 points on the ground
% Inputs:
%    H - the computed homography from the corners in the image
%    render_points - size (N x 3) matrix of world points to project
%    K - size (3 x 3) calibration matrix for the camera
% Outputs: 
%    proj_points - size (N x 2) matrix of the projected points in pixel
%      coordinates
%    t - size (3 x 1) vector of the translation of the transformation
%    R - size (3 x 3) matrix of the rotation of the transformation
% Written by Stephen Phillips for the Coursera Robotics:Perception course

% YOUR CODE HERE: Extract the pose from the homography
H_prime=inv(K)*H;

if H(3,3)>0
  R_prime=H_prime(:,1:2);
  R_prime=cat(2, R_prime,cross(H_prime(:,1),H_prime(:,2)));
else
  R_prime=-H(:,1:2);
  R_prime=cat(2, R_prime,cross(H_prime(:,1),H_prime(:,2)));
end

t=H_prime(:,3)/norm(H_prime(:,1));
[U,S,V]=svd(R_prime);
S=[1 0 0;0 1 0;0 0 det(U*V')];
R=U*S*V';
det(R)

% YOUR CODE HERE: Project the points using the pose
%temp=cat(2, R, t);
%proj_points=K*temp*render_points;
temp=R*render_points'+t;
proj_points=temp./temp(size(temp,1),:);
proj_points=(proj_points(1:2,:))';

end
