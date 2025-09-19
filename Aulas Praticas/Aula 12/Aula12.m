%% Aula 12
%MAgner Gusse 110180

%% Ex1 Criar mapa com obstaculos
clc
close
clear

 workspace = polyshape([0 10 10 0], [0 0 10 8]);
 obs(1)=polyshape([2 4 4 2], [2 2 4 4]);
 obs(2)=polyshape([5 7.5 7.5 5], [6 5 7 8]);
 obs(3)=polyshape([6 7 8 7], [2 3 2 1]);
 obstacles = polyshape(); % initial empty list
 for k=1:numel(obs)
 obstacles = union(obstacles,obs(k)); %join all obs
 end
 plot(workspace, 'FaceColor','none')
 axis equal, hold on
 plot(obstacles, 'FaceColor', [0.5 0.5 0.5])
 hold on

 %% Ex 2

  free_space = workspace;
 for i = 1:length(obs)
 free_space = subtract(free_space, obs(i));
 end
 plot(free_space,'FaceColor','none','EdgeColor', 'c','LineWidth',4);
hold on
 %% Ex 3

 verts=free_space.Vertices;
 %all vertices
 verts=verts(all(isfinite(verts),2),:);%removes NaN
 verts=unique(verts, 'rows');%no duplicates
 plot(verts(1,:), verts(2,:),'*r') %optional plot
 dt = delaunayTriangulation(verts);%the Delaunay
 hDT=triplot(dt); %display it

 %% Ex 4

  % Obtain the full list of triangle centers
 triCenters = dt.incenter;

 % Obtain the list of the overlaping triangles
 % 1-list of logical values whether inside or outside free
 inside = isinterior(free_space, triCenters(:,1), triCenters(:,2));

 % 2-With the list extract the valid triangles only
 validTriangles = dt.ConnectivityList(inside, :);
 %pause %wait for key press before deleting previous graph
 hDT.delete %delete previous plot to show next plot

 %and plot the valid triangles only
 triplot(validTriangles, dt.Points(:,1), dt.Points(:,2));

 %% Ex 5

  numTri = size(validTriangles,1);
 adj = zeros(numTri);
 % number of triangles
 % init adjacency matrix
 % Two triangles are adjacent if they share an edge, i.e. 2 points
 for i = 1:numTri
         for j = i+1:numTri
         shared = intersect(validTriangles(j,:), validTriangles(i,:));
                 if numel(shared) == 2
                 adj(i,j) = 1;
                 adj(j,i) = 1;
                 end
         end
 end

 %% Ex 6

  centroids = triCenters(inside, :);
  g = graph(adj);
 hG = plot(g, 'XData', centroids(:,1), 'YData', centroids(:,2));
hold on
 %% Ex 7

  startNode=2;
 endNode=18;
 [path1, pathLength] = shortestpath(g, startNode, endNode);
 highlight(hG,path1,'EdgeColor', 'magenta','LineWidth', 5)
hold on
 %% Ex 8

  D=zeros(height(centroids));
 for i=1:height(centroids)
 for j=1:height(centroids)
 D(i,j)=norm(centroids(i,:)-centroids(j,:));
 end
 end


 % ha solucoes mais compactas
 % D(i,j) Euclidean distance between node i and j

 % Depois basta combinar as matrizes desta forma:
 % Keep distances only for connected node pairs
 adjc = D .* adj;
 g2=graph(adjc);
 
  [path2, pathLength] = shortestpath(g2, startNode, endNode);
  highlight(hG,path2,'EdgeColor', 'cyan','LineWidth', 2)
  hold on
  %% EX 9

SS=[0.5 3]; FF=[8.5 8];



 g2.Nodes.Name = string(1:numnodes(g2))';
 g2 = addnode(g2, 'S');
 g2 = addnode(g2, 'F');
 
 Diststart=vecnorm(centroids-SS,1,2);
 nearStartID= find(Diststart==min(Diststart));

 centroids=[centroids;SS];
 
 Distend=vecnorm(centroids-FF,1,2);
 nearEndID= find(Distend==min(Distend));
 centroids=[centroids;FF];
 

 % calculate IDs of closest nodes to 'S' and to 'F'
 g2 = addedge(g2, ""+nearStartID, 'S', norm(centroids(nearStartID,:)- SS));
 g2 = addedge(g2, ""+nearEndID, 'F', norm(centroids(nearEndID,:)- FF));
 
 hG2=plot(g2, 'XData', centroids(:,1), 'YData', centroids(:,2));
 newStartNode='S'; newEndNode='F';
 [path3, pathLength] = shortestpath(g2, newStartNode, newEndNode);
 highlight(hG2,path3,'EdgeColor','g','LineWidth',3)

 %% Ex 10

  %workspace =
 %obstacles =
 res = 0.2;
 %defined earlier
 %defined earlier
 %grid resolution
 ext=max(workspace.Vertices);%maximum limits of workspace
 x = 0:res:ext(1);
 %span of x coordinate
 y = 0:res:ext(2);
 rows = numel(y);
 cols = numel(x);
 %span of y coordinate
 %number of rows of grid
 %number of columns of grid
 occGrid = false(***, ***); %initial empty occupancy grid (0=free, 1=occupied)


 %% Ex 11
  [X, Y] = meshgrid(x, y);
 for i = 1:rows
 for j = 1:cols
 %All combinations of x and y coordinates
 %row index
 %column index
 pt = [X(i,j), Y(i,j)]; %actual metric coordinates of grid cell (i,j)
 if isinterior(workspace, pt) && isinterior(obstacles, pt)
 occGrid(i,j) = ***; %empty (free space)
 else
 occGrid(i,j) = ***; %occupied (workspace limits, obstacles)
 end
 end
 end

 figure
 hold on; axis equal tight;
 colormap([1 1 1; 0.5 0.5 0.5]);%white and gray
 imagesc(x, y, occGrid);
 %display occGrid as an image