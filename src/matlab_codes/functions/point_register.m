function [R,t,FRE,FREcomponents] = point_register(X,Y,w,n_t)
% [R,t,FRE,FREcomponents] = POINT_REGISTER(X,Y,w,n_t)
% Find R,t such that R*X + t*ones(1,N) ~ Y,
% where N = number of columns of X and Y.
% Author: J. Michael Fitzpatrick
% Each column of X and each column of Y represents a
% K-dimensional point, where K is the number of rows.
% X and Y must have the same number of points (columns).
% FRE = RMS fiducial registration error.
% FREcomponents = R*X + t*ones(1,N) ~ Y.
% Uses algorithm 8.1 and notation of pp. 469-70 of
% J. Michael Fitzpatrick, Derek L. G. Hill, and Calvin R. Maurer, Jr.,
% "Image Registration", Chapter 8 of "Handbook of Medical Imaging,
% Volume 2, Medical Image Processing and Analysis",
% Milan Sonka and J. Michael Fitzpatrick, eds., SPIE Press,
% Bellingham, Wa, 2000.
%
% Sep 18, 2005:
% Modified by JMF to allow arb. value for K. In
% particular, it is able to handle K = 1 or 2, as well as K = 3, but it is
% written to handle any number of dimensions. The method of
% insuring that R is proper (by means of the diagonal matrix D) has
% been proven to find the optimum transformation for up to 3 dimensions.
%
% May 8, 2007:
% Modified by JMF to allow N < K. This is a trivial
% modification: Removal of a check for N<K. The result is,
% for example, that a registration can be found for two points
% in three-dimensional space (N = 2, K = 3).
%
% May 8, 2007:
% Also modified by JMF to add the optional fourth argument.
% This argument is the number of points (beginning with the first)
% for which translation is to be calculated and used. For the rest
% of the points only rotation is used. This feature makes new
% applications possible. For example, if it is desired to use
% a set of points for which the first subset are real points,
% but the second subset are unit vectors representing direction
% only, then n_t is the number of real points.
%
% February 18, 2010
% Modified by JMF to add the option of isotropic weighting, also from the
% reference given above. The option was tested by Ramya Balachandran on
% another version of point_register, which allowed a set of squared
% weights to be given as the third argument. 

if nargin < 2
    error('At least two input arguments are required.');
end
[K N] = size(X);
[K_Y N_Y] = size(Y);
if K ~= K_Y
    error('X and Y must have the same number of rows.')
elseif (K ~= K_Y) || (N ~= N_Y)
    error('X and Y must have the same number of columns.');
end
if nargin < 3
    n_t = N; % User does not want to force any of t to zero.
end
if n_t > N
    error('n_t cannot be greater than N.');
end
if n_t == 0
    Xbar = zeros(K,1); Ybar = zeros(K,1); % dummy centroids
else
    X_n_t = X(:,1:n_t); % the first n_t columns
    Y_n_t = Y(:,1:n_t); % the first n_t columns
    Xbar = mean(X_n_t,2);  % X centroid
    Ybar = mean(Y_n_t,2);  % Y centroid
end
Xtilde = X-repmat(Xbar,1,N); % X relative to centroid
Ytilde = Y-repmat(Ybar,1,N); % Y relative to centroid
Xtilde(:,n_t+1:N) = X(:,n_t+1:N);
Ytilde(:,n_t+1:N) = Y(:,n_t+1:N);
H = Xtilde*Ytilde';  % cross covariance matrix
[U S V] = svd(H);    % U*S*V' = H
D = diag([ones(1,K-1), det(V*U)]); % used to insure that R is proper
R = V*D*U';
t = Ybar-R*Xbar;
if nargout >= 3
    if n_t ~= N
        T = [repmat(t,1,n_t),zeros(K,N-n_t)];
    else
        T = repmat(t,1,N);
    end
    FREcomponents = R*X + T - Y;
    FRE = sqrt(mean(sum(FREcomponents.^2,1)));
end




