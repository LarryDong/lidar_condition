% Load the ROS bag file

file_height0 = "E:\lidar_mount_height_data\data\0\2024-04-18-17-33-10.bag";
file_height300 = "E:\lidar_mount_height_data\data\300\2024-04-22-10-43-13.bag";
test_file = "C:\Users\larrydong\Desktop\multi-box.bag";

fprintf("--> Loading rosbag... \n");
bag = rosbag(test_file);
pointCloudTopic = select(bag, 'Topic', '/ouster/points');
msgs = readMessages(pointCloudTopic);
fprintf("<-- Data loaded. \n")

% Processing each PointCloud2 message
total_scan = length(msgs);
fprintf("Total scan number: %d \n", total_scan);


% Preallocate the array for storing condition numbers
cond_rr = [];
cond_tt = [];


for i = 1:total_scan
	% if(mod(i, 10)~=0)
	% 	continue;
	% end
	fprintf("  process: %d / %d, \n", i, total_scan);
    ptCloudMsg = msgs{i};

    % Convert PointCloud2 ROS message to MATLAB point cloud object
    ptCloud = pointCloud(readXYZ(ptCloudMsg));

	% show pc
	% pcshow(ptCloud);

	% continue;

    % Number of points in the cloud
    numPoints = ptCloud.Count;

    % Preallocate A matrix
    A = zeros(6, 6);

    % Loop over each point in the point cloud
    for j = 1:numPoints

		if mod(j, 100)~=0
			continue;
		end
		
		p = ptCloud.Location(j, :);
		if (p(1) == 0 || p(2) ==0 || p(3) == 0)
			continue;
		end
		if (sqrt(p(1)^2+p(2)^2+p(3)^2) > 100)
			continue;
		end

        % Find K nearest neighbors (K=5)
		K = 10;
        [indices, ~] = findNearestNeighbors(ptCloud, p, K);

        % Get coordinates of the nearest neighbors
        KPoints = ptCloud.Location(indices, :);

		% [~, ~, normal] = pca(KPoints);		% ISSUES.
		meanKPoints = mean(KPoints, 1);
		covMatrix = (KPoints - meanKPoints)' * (KPoints - meanKPoints) / (size(KPoints, 1) - 1);
		
		% Eigenvalue decomposition to find the normal vector
		[V, D] = eig(covMatrix);
		
		% The normal vector is the eigenvector associated with the smallest eigenvalue
		[~, idx] = min(diag(D));
		normal = V(:, idx);

        % Calculate the cross product of point position and normal vector
        p_cross_n = cross(ptCloud.Location(j, :), normal);

        % Construct Hi matrix
        Hi = [p_cross_n, normal'];
        
        % Accumulate Hi' * Hi
        A = A + Hi' * Hi;
    end

    % Extract Arr and Att blocks
    Arr = A(1:3, 1:3);
    Att = A(4:6, 4:6);

    % Singular Value Decomposition to find the eigenvalues
    [~, S_rr, ~] = svd(Arr);
    [~, S_tt, ~] = svd(Att);

    % Calculate the condition numbers
    cond_rr = [cond_rr, max(diag(S_rr)) / min(diag(S_rr))];
    cond_tt = [cond_tt, max(diag(S_tt)) / min(diag(S_tt))];
end

% Output or use the condition numbers as needed
disp('Condition numbers for Arr:');
disp(cond_rr);
disp('Condition numbers for Att:');
disp(cond_tt);

filename = 'multibox-test-single-ds10.mat';
% Save cond_rr and cond_tt to a .mat file
save(filename, 'cond_rr', 'cond_tt');