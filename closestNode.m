function [node] = closestNode(path, q)
%
% INPUTS:
%   path  - Nx2 vector. either startPath or goalPath.
%   q     - 1x2 matrix. random node generated. (yet to be validated)
%   
% OUTPUTS:
%   node  - 1x2 matrix. The closest position to the random node within
%           the prespecified path.


%% Closest Node

    % Determine Current Number of Configurations in Path
    [configs, ~] = size(path);

    diffOld = 10000;
    % Find Closest Node to q in Specified Path
    for i=1:1:configs
        closeNode = path(i,:);
        diffNew = norm(q - closeNode); % Find distance across all joint vals
        if (diffNew <= diffOld)  % if new existing node is closer
            diffOld = diffNew;  % redefine the old node to be the new
            bestNode = closeNode;  % Declare this new node the closest
        end
    end
    node = bestNode;

end