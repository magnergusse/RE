function path = a_star_search(grid, start, goal)
    rows = size(grid,1);
    cols = size(grid,2);

    % Create cost and heuristic matrices
    g = Inf(rows, cols);
    f = Inf(rows, cols);
    g(start(1), start(2)) = 0;
    f(start(1), start(2)) = heuristic(start, goal);

    % Closed set
    closed = false(rows, cols);

    % Parent tracking
    parent = zeros(rows, cols, 2);

    % Open set
    open = start;
    
    while ~isempty(open)
        % Get node with lowest f
        f_vals = arrayfun(@(i) f(open(i,1), open(i,2)), 1:size(open,1));
        [~, idx] = min(f_vals);
        current = open(idx,:);
        open(idx,:) = [];

        if isequal(current, goal)
            % Reconstruct path
            path = current;
            while ~isequal(current, start)
                current = squeeze(parent(current(1), current(2), :))';
                path = [current; path];
            end
            return
        end

        closed(current(1), current(2)) = true;

        % 4-connected neighbors
        neighbors = [0 -1; 0 1; -1 0; 1 0];
        for i = 1:4
            neighbor = current + neighbors(i,:);
            if neighbor(1) < 1 || neighbor(1) > rows || ...
               neighbor(2) < 1 || neighbor(2) > cols
                continue
            end
            if closed(neighbor(1), neighbor(2)) || grid(neighbor(1), neighbor(2))
                continue
            end
            tentative_g = g(current(1), current(2)) + 1;

            if tentative_g < g(neighbor(1), neighbor(2))
                parent(neighbor(1), neighbor(2), :) = current;
                g(neighbor(1), neighbor(2)) = tentative_g;
                f(neighbor(1), neighbor(2)) = tentative_g + heuristic(neighbor, goal);
                if ~ismember(neighbor, open, 'rows')
                    open = [open; neighbor];
                end
            end
        end
    end
    path = []; % No path found
end

function h = heuristic(a, b)
    h = norm(a - b); % Euclidean distance
end
