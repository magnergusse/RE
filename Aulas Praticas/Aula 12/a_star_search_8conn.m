function path = a_star_search_8conn(grid, start, goal)
    [rows, cols] = size(grid);
    g = Inf(rows, cols);
    f = Inf(rows, cols);
    g(start(1), start(2)) = 0;
    f(start(1), start(2)) = heuristic(start, goal);

    closed = false(rows, cols);
    parent = zeros(rows, cols, 2);
    open = start;

    while ~isempty(open)
        f_vals = arrayfun(@(i) f(open(i,1), open(i,2)), 1:size(open,1));
        [~, idx] = min(f_vals);
        current = open(idx,:);
        open(idx,:) = [];

        if isequal(current, goal)
            path = current;
            while ~isequal(current, start)
                current = squeeze(parent(current(1), current(2), :))';
                path = [current; path];
            end
            return;
        end

        closed(current(1), current(2)) = true;

        % 8-connected neighbors
        for dx = -1:1
            for dy = -1:1
                if dx == 0 && dy == 0, continue; end
                neighbor = current + [dy, dx];
                if any(neighbor < 1) || neighbor(1) > rows || neighbor(2) > cols
                    continue;
                end
                if closed(neighbor(1), neighbor(2)) || grid(neighbor(1), neighbor(2))
                    continue;
                end
                cost = norm([dx dy]);  % 1 for axis, sqrt(2) for diagonal
                tentative_g = g(current(1), current(2)) + cost;
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
    end
    path = [];
end

function h = heuristic(a, b)
    h = norm(a - b); % Euclidean distance
end
