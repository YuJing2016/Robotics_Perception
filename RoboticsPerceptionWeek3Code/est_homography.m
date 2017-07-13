function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
H = [];
A = [];

%wrap data points up to form a matrix
for i = 1:4

  x1=video_pts(i,1);
  x2=video_pts(i,2);
  
  x1_p=logo_pts(i,1);
  x2_p=logo_pts(i,2);

  temp=[-x1 -x2 -1 0 0 0 x1*x1_p x2*x1_p x1_p; 0 0 0 -x1 -x2 -1 x1*x2_p x2*x2_p x2_p];
  
  A=cat(1, A, temp);
  
end 

[~, ~, V]=svd(A);

%reshape V

H=(reshape(V(:,9), 3,3))';

end

