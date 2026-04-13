function [pc_clean, features, labels, pipeline_info] = lidar_point_cloud( ...
    params, points_3d, reflectivity, inject_errors)
% LIDAR_POINT_CLOUD  Full point-cloud processing pipeline.
%
%   [pc_clean, features, labels, pipeline_info] = ...
%       lidar_point_cloud(params, points_3d, reflectivity)
%
%   Pipeline stages (per PDF §VII-A pseudocode):
%       P_filtered    ← RemoveNoise(P_raw)
%       P_ground      ← GroundSegmentation(P_filtered)
%       P_objects     ← P_filtered − P_ground
%       F             ← ExtractFeatures(P_objects)
%       C             ← Classify(F)
%       P_classified  ← Combine(P_ground, C)
%
%   Extracted features (PDF §VII-B + implementation extension):
%       1. Height above ground          (PDF §VII-B)
%       2. Point density                (implementation extension)
%       3. Spatial distribution variance (PDF §VII-B)
%       4. Reflectivity intensity        (PDF §VII-B, Eq. 4)
%       5. Geometric moments             (PDF §VII-B)
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
    % Warn if point cloud is very large (O(N²) memory)
    if N > 5000
        warning('lidar_point_cloud:largeN', ...
            'Point cloud has %d points — distance matrix requires %.0f MB.', ...
            N, N^2 * 8 / 1e6);
    end
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

% Track which rows of the original input (points_3d / reflectivity) each
% object point came from.  Two-step: noise filter then ground filter.
%   Step 1: inlier_mask maps filtered rows → original rows
%   Step 2: ~ground_mask maps object rows → filtered rows
inlier_idx              = find(inlier_mask);          % in original coords
obj_in_filtered         = find(~ground_mask);         % in filtered coords
pipeline_info.obj_input_idx = inlier_idx(obj_in_filtered);  % in original coords

%% =======================================================================
%  STAGE 3: Object Isolation  (already done above)
%  =======================================================================
M_obj = size(pc_objects, 1);

%% =======================================================================
%  STAGE 4: Feature Extraction
%  =======================================================================
%  5 features per-point (PDF §VII-B), extracted from local neighbourhood.
%  A data-adaptive radius is used so features are meaningful regardless
%  of overall point-cloud density (e.g., sparse drone scans).

features = zeros(M_obj, 5);

if M_obj > 0
    % --- Feature 1: Height above the fitted ground plane ---
    features(:, 1) = abs(pc_objects * n_vec' + d_plane);

    if M_obj > 1
        % Pre-compute full pairwise distance matrix (used for all k-NN features)
        D_obj = compute_dist_matrix(pc_objects);       % [M_obj x M_obj]

        % Sort each row once; re-use for all neighbourhood lookups
        D_sorted = sort(D_obj, 2, 'ascend');           % col 1 = self (0)

        k_local = min(10, M_obj - 1);                 % neighbours to use

        % Data-adaptive density radius: 25th percentile of each point's
        % k-NN distance, so density is non-trivial even for sparse clouds.
        r_density = max(median(D_sorted(:, min(k_local+1, end))), 1e-6);

        % --- Feature 2: Point density (neighbours within r_density) ---
        features(:, 2) = max(sum(D_obj < r_density, 2) - 1, 0);

        % --- Features 3 & 5: spatial variance and geometric moment ---
        for i = 1:M_obj
            nn_cols  = 2 : min(k_local + 1, M_obj);   % skip self (col 1)
            [~, ord] = sort(D_obj(i, :), 'ascend');
            nn_idx   = ord(nn_cols);

            local_pts = pc_objects(nn_idx, :);         % [k x 3]

            if size(local_pts, 1) >= 2
                % Feature 3: trace of local covariance (spread)
                features(i, 3) = trace(cov(local_pts));
            else
                % Single neighbour: use squared distance as variance proxy
                features(i, 3) = D_sorted(i, 2)^2;
            end

            % Feature 5: mean squared distance to local centroid
            centroid       = mean(local_pts, 1);
            diffs          = local_pts - centroid;
            features(i, 5) = mean(sum(diffs.^2, 2));
        end
    else
        % Only one object point: all neighbourhood features are 0
        features(1, 2) = 0;
        features(1, 3) = 0;
        features(1, 5) = 0;
    end

    % --- Feature 4: Reflectivity intensity (normalised, from channel) ---
    features(:, 4) = refl_obj(:);

    % Clamp all features to non-negative (physical constraint)
    features = max(features, 0);
end

%% =======================================================================
%  STAGE 5: Classification
%  =======================================================================
%  Classification is now handled by lidar_classifier.m (PDF §VII-C)
%  which implements a proper Random Forest with training data generation,
%  TreeBagger (when available), and Table V environmental evaluation.
%  This module only does feature extraction; labels come from the
%  classifier called in run_lidar_simulation.m.

labels = [];   % placeholder — filled by lidar_classifier.m

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
