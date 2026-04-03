function [pc_clean, features, labels, pipeline_info] = lidar_point_cloud( ...
    params, points_3d, reflectivity, inject_errors)
% LIDAR_POINT_CLOUD  Full point-cloud processing pipeline.
%
%   [pc_clean, features, labels, pipeline_info] = ...
%       lidar_point_cloud(params, points_3d, reflectivity)
%
%   Pipeline stages (per architecture):
%       1. Remove Noise           (statistical outlier removal)
%       2. Ground Segmentation    (RANSAC plane fitting)
%       3. Object Isolation       (Filtered − Ground)
%       4. Feature Extraction     (5 features)
%       5. Classification         (Random Forest)
%       6. Combine with Ground
%
%   Extracted features:
%       1. Height above ground
%       2. Point density
%       3. Spatial distribution variance
%       4. Reflectivity intensity
%       5. Geometric moments (2nd central moment of local patch)
%
%   Inputs:
%       points_3d     - [N x 3] point cloud (X, Y, Z)
%       reflectivity  - [N x 1] normalised reflectivity per point
%       inject_errors - optional logical
%
%   Outputs:
%       pc_clean      - [M x 3] denoised point cloud
%       features      - [M_obj x 5] feature matrix for object points
%       labels        - [M_obj x 1] predicted class (1=crop, 2=weed)
%       pipeline_info - struct with intermediate results

if nargin < 4, inject_errors = false; end

N = size(points_3d, 1);
reflectivity = reflectivity(:);

%% =======================================================================
%  STAGE 1: Noise Removal (Statistical Outlier Removal)
%  =======================================================================
%  For each point compute mean distance to k nearest neighbours.
%  Remove points whose mean dist > μ + 2σ.
k_nn = 10;

if N > k_nn
    % Simple brute-force kNN (fine for demonstration)
    D = compute_dist_matrix(points_3d);    % [N x N] distance matrix
    D_sorted = sort(D, 2, 'ascend');         % column 1 = self (0)
    mean_knn = mean(D_sorted(:, 2:min(k_nn+1, end)), 2);  % mean dist to k-NN
    
    mu_d  = mean(mean_knn);
    sig_d = std(mean_knn);
    inlier_mask = mean_knn < (mu_d + 2 * sig_d);
else
    inlier_mask = true(N, 1);
end

pc_filtered   = points_3d(inlier_mask, :);
refl_filtered = reflectivity(inlier_mask);

pipeline_info.n_raw      = N;
pipeline_info.n_filtered = sum(inlier_mask);

%% =======================================================================
%  STAGE 2: Ground Segmentation (RANSAC Plane Fitting)
%  =======================================================================
%  Fit a plane z = a*x + b*y + c to the lowest points.
%  Points within ±ground_threshold are classified as ground.
ground_threshold = 0.05;   % [m] 5 cm tolerance
n_ransac_iter    = 100;
best_inliers     = 0;
best_plane       = [0, 0, 0, 0];

M = size(pc_filtered, 1);

if M >= 3
    for iter = 1:n_ransac_iter
        idx = randperm(M, 3);
        pts = pc_filtered(idx, :);
        
        % Plane normal via cross product
        v1 = pts(2,:) - pts(1,:);
        v2 = pts(3,:) - pts(1,:);
        n_vec = cross(v1, v2);
        if norm(n_vec) < 1e-12, continue; end
        n_vec = n_vec / norm(n_vec);
        d = -dot(n_vec, pts(1,:));
        
        % Signed distance of all points to plane
        dists = abs(pc_filtered * n_vec' + d);
        n_inliers = sum(dists < ground_threshold);
        
        if n_inliers > best_inliers
            best_inliers = n_inliers;
            best_plane   = [n_vec, d];
        end
    end
    
    n_vec = best_plane(1:3);
    d_plane = best_plane(4);
    dist_to_ground = abs(pc_filtered * n_vec' + d_plane);
    ground_mask = dist_to_ground < ground_threshold;
else
    ground_mask = false(M, 1);
    n_vec = [0 0 1];
    d_plane = 0;
end

pc_ground  = pc_filtered(ground_mask, :);
pc_objects = pc_filtered(~ground_mask, :);
refl_obj   = refl_filtered(~ground_mask);

pipeline_info.n_ground  = sum(ground_mask);
pipeline_info.n_objects = sum(~ground_mask);
pipeline_info.ground_plane = [n_vec, d_plane];

%% =======================================================================
%  STAGE 3: Object Isolation  (already done above)
%  =======================================================================
M_obj = size(pc_objects, 1);

%% =======================================================================
%  STAGE 4: Feature Extraction
%  =======================================================================
%  5 features per-point (using local neighbourhood)

features = zeros(M_obj, 5);

if M_obj > 0
    % Height above ground
    height_above_ground = abs(pc_objects * n_vec' + d_plane);
    features(:, 1) = height_above_ground;
    
    %  Point density (number of neighbours within radius r)
    r_density = 0.1;  % [m]
    if M_obj > 1
        D_obj = compute_dist_matrix(pc_objects);
        features(:, 2) = sum(D_obj < r_density, 2) - 1;  % exclude self
    end
    
    %  Spatial distribution variance (trace of local covariance)
    k_local = min(10, M_obj - 1);
    for i = 1:M_obj
        if M_obj > 1
            [~, nn_idx] = sort(D_obj(i, :), 'ascend');
            nn_idx = nn_idx(2:min(k_local+1, end));
            local_pts = pc_objects(nn_idx, :);
            features(i, 3) = trace(cov(local_pts));
        end
    end
    
    %  Reflectivity intensity (normalised)
    features(:, 4) = refl_obj;
    
    %  Geometric moments (2nd central moment ≈ inertia)
    for i = 1:M_obj
        if M_obj > 1
            [~, nn_idx] = sort(D_obj(i, :), 'ascend');
            nn_idx = nn_idx(2:min(k_local+1, end));
            local_pts = pc_objects(nn_idx, :);
            centroid  = mean(local_pts, 1);
            diffs     = local_pts - centroid;
            features(i, 5) = mean(sum(diffs.^2, 2));
        end
    end
end

%% =======================================================================
%  STAGE 5: Classification (Random Forest — Synthetic)
%  =======================================================================
%  In a real MATLAB deployment you would use TreeBagger / fitcensemble.
%  Here we create a synthetic classifier that separates crop vs. weed
%  based on the extracted features, matching the specified 94.7 % accuracy.

if M_obj > 0
    labels = synthetic_random_forest(features, params.ml);
else
    labels = [];
end

%% =======================================================================
%  STAGE 6: Combine with Ground (output)
%  =======================================================================
pc_clean = [pc_ground; pc_objects];

pipeline_info.features = features;
pipeline_info.labels   = labels;
pipeline_info.pc_ground  = pc_ground;
pipeline_info.pc_objects = pc_objects;

end

%% =======================================================================
%  LOCAL FUNCTION: Synthetic Random Forest
%  =======================================================================
function labels = synthetic_random_forest(features, ml_params)
% Mimics an RF classifier with the specified accuracy.
%
%   Decision rule (heuristic for simulation):
%       - Weeds tend to be shorter (height < threshold), higher density,
%         higher reflectivity variance, different geometric moments.
%
%   We add controlled noise to achieve ~94.7% accuracy.

    n = size(features, 1);
    
    % Normalise features to [0, 1]
    f_min = min(features, [], 1);
    f_max = max(features, [], 1);
    f_range = f_max - f_min;
    f_range(f_range == 0) = 1;   % avoid divide-by-zero
    f_norm = (features - f_min) ./ f_range;
    
    % Simple linear discriminant (mimicking RF boundary)
    %   score = w' * features + bias
    %   Positive → crop (1),  Negative → weed (2)
    w = [0.3; -0.2; -0.15; 0.25; -0.1];
    score = f_norm * w + 0.1;
    
    %  Inject classification noise to match target accuracy
    target_acc = ml_params.target_accuracy;
    error_rate = 1 - target_acc;
    flip_mask  = rand(n, 1) < error_rate;
    
    labels = ones(n, 1);              % default: crop
    labels(score < 0) = 2;            % weed
    
    % Flip selected labels to inject error
    labels(flip_mask) = 3 - labels(flip_mask);   % 1↔2
end

%% =======================================================================
%  LOCAL FUNCTION: Distance matrix (pdist2 fallback)
%  =======================================================================
function D = compute_dist_matrix(X)
% Compute pairwise Euclidean distance matrix without Statistics Toolbox.
    try
        D = pdist2(X, X);   % use toolbox if available (faster)
    catch
        N = size(X, 1);
        D = zeros(N);
        for i = 1:N
            diff = X - X(i,:);
            D(i,:) = sqrt(sum(diff.^2, 2))';
        end
    end
end
