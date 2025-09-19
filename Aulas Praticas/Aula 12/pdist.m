function Y = pdist(X,dist,varargin)
%PDIST Pairwise distance between observations.
%   D = PDIST(X) returns a vector D containing the Euclidean distances
%   between each pair of observations in the M-by-N data matrix X. Rows of
%   X correspond to observations, columns correspond to variables. D is a
%   1-by-(M*(M-1)/2) row vector, corresponding to the M*(M-1)/2 pairs of
%   observations in X.
%
%   D = PDIST(X, DISTANCE) computes D using DISTANCE.  Choices are:
%
%       'euclidean'        - Euclidean distance (default)
%       'squaredeuclidean' - Squared Euclidean distance 
%       'seuclidean'       - Standardized Euclidean distance. Each
%                            coordinate difference between rows in X is
%                            scaled by dividing by the corresponding
%                            element of the standard deviation S=NANSTD(X).
%                            To specify another value for S, use
%                            D=PDIST(X,'seuclidean',S).
%       'fasteuclidean'    - Euclidean distance computed by using an
%                            alternative algorithm that saves time. This
%                            faster algorithm can, in some cases, reduce
%                            accuracy.
%       'fastsquaredeuclidean'
%                          - Squared Euclidean distance computed by using 
%                            an alternative algorithm that saves time. This
%                            faster algorithm can, in some cases, reduce
%                            accuracy.
%       'fastseuclidean'   - Standardized Euclidean distance computed by 
%                            using an alternative algorithm that saves 
%                            time. This faster algorithm can, in some 
%                            cases, reduce accuracy.
%       'cityblock'        - City Block distance
%       'minkowski'        - Minkowski distance. The default exponent is 2.
%                            To specify a different exponent, use
%                            D = PDIST(X,'minkowski',P), where the exponent
%                            P is a scalar positive value.
%       'chebychev'        - Chebychev distance (maximum coordinate
%                            difference)
%       'mahalanobis'      - Mahalanobis distance, using the sample
%                            covariance of X as computed by NANCOV. To
%                            compute the distance with a different
%                            covariance, use
%                            D =  PDIST(X,'mahalanobis',C), where the
%                            matrix C is symmetric and positive definite.
%       'cosine'           - One minus the cosine of the included angle
%                            between observations (treated as vectors)
%       'correlation'      - One minus the sample linear correlation
%                            between observations (treated as sequences of
%                            values).
%       'spearman'         - One minus the sample Spearman's rank 
%                            correlation between observations (treated as
%                            sequences of values).
%       'hamming'          - Hamming distance, percentage of coordinates
%                            that differ
%       'jaccard'          - One minus the Jaccard coefficient, the
%                            percentage of nonzero coordinates that differ
%       function           - A distance function specified using @, for
%                            example @DISTFUN.
%
%   A distance function must be of the form
%
%         function D2 = DISTFUN(XI, XJ),
%
%   taking as arguments a 1-by-N vector XI containing a single row of X, an
%   M2-by-N matrix XJ containing multiple rows of X, and returning an
%   M2-by-1 vector of distances D2, whose Jth element is the distance
%   between the observations XI and XJ(J,:).
%
%   The output D is arranged in the order of ((2,1),(3,1),..., (M,1),
%   (3,2),...(M,2),.....(M,M-1)), i.e. the lower left triangle of the full
%   M-by-M distance matrix in column order.  To get the distance between
%   the Ith and Jth observations (I < J), either use the formula
%   D((I-1)*(M-I/2)+J-I), or use the helper function Z = SQUAREFORM(D),
%   which returns an M-by-M square symmetric matrix, with the (I,J) entry
%   equal to distance between observation I and observation J.
%
%   D = PDIST(X,DISTANCE,'CacheSize',CACHESIZE) uses an intermediate matrix 
%   stored in cache to compute D, when 'Distance' is one of 
%   {'fasteuclidean','fastsquaredeuclidean','fastseuclidean'}. 'CacheSize'
%   can be a positive scalar or 'maximal'. The default is 1e3.
%   If numeric, 'CacheSize' specifies the cache size in megabytes (MB) to
%   allocate for an intermediate matrix.
%   If 'maximal', pdist attempts to allocate enough memory for an entire
%   intermediate matrix whose size is M-by-M (M is the number of rows of
%   the input data X).
%   'CacheSize' does not have to be large enough for an entire intermediate
%   matrix, but it must be at least large enough to hold an M-by-1 vector. 
%   Otherwise, the regular algorithm of computing Euclidean distance will
%   be used instead. If the specified cache size exceeds the available
%   memory, MATLAB issues an out-of-memory error.
%
%   Example:
%      % Compute the ordinary Euclidean distance
%      X = randn(100, 5);                 % some random points
%      D = pdist(X, 'euclidean');         % euclidean distance
%
%      % Compute the Euclidean distance with each coordinate difference
%      % scaled by the standard deviation
%      Dstd = pdist(X,'seuclidean');
%
%      % Use a function handle to compute a distance that weights each
%      % coordinate contribution differently
%      Wgts = [.1 .3 .3 .2 .1];           % coordinate weights
%      weuc = @(XI,XJ,W)(sqrt((XI-XJ).^2 * W'));
%      Dwgt = pdist(X, @(Xi,Xj) weuc(Xi,Xj,Wgts));
%
%   See also SQUAREFORM, LINKAGE, SILHOUETTE, PDIST2.

%   Copyright 1993-2022 The MathWorks, Inc.

if nargin > 1
    dist = convertStringsToChars(dist);
end

if nargin > 2
    [varargin{:}] = convertStringsToChars(varargin{:});
end

additionalArg = [];
if nargin < 2
    dist = 'euc';
else
    if ischar(dist)
        methods = {'euclidean'; 'seuclidean'; 'cityblock'; 'chebychev'; ...
            'mahalanobis'; 'minkowski'; 'cosine'; 'correlation'; ...
            'spearman'; 'hamming'; 'jaccard';'squaredEuclidean';...
            'fasteuclidean';'fastseuclidean';'fastsquaredeuclidean'};
        i = find(strncmpi(dist,methods,length(dist)));
        if length(i) > 1
            error(message('stats:pdist:AmbiguousDistance', dist));
        elseif isempty(i)
            % Assume an unrecognized string is a user-supplied distance
            % function name, change it to a handle.
            distfun = str2func(dist);
            distargs = varargin;
            dist = 'usr';
        else
            if i == 12 %'squaredeuclidean'
                dist = 'sqe'; % represent squared Euclidean
            elseif i==13
                dist = 'feu'; % represent fast Euclidean
            elseif i==14
                dist = 'fse'; % represent fast Standardized Euclidean
            elseif i==15
                dist = 'fqe'; % represent fast squared Euclidean
            else
                dist = lower(methods{i}(1:3));
            end

            if ~isempty(varargin)
                arg = varargin{1};
                
                % Get the additional distance argument from the inputs
                if isnumeric(arg)
                    switch dist
                        case {'seu' 'mah' 'min' 'fse'}
                            additionalArg = arg;
                            varargin = varargin(2:end);
                    end
                end
            end

        end
    elseif isa(dist, 'function_handle') ||  isa(dist, 'inline')
        distfun = dist;
        distargs = varargin;
        dist = 'usr';
    else
        error(message('stats:pdist:BadDistance'));
    end
end



% Sanity check for CacheSize
if ismember(dist, {'feu' 'fse' 'fqe'})
    pnames = {'cachesize'};
    dflts =  {1e3};
    cacheSize = statslib.internal.parseArgs(pnames, dflts, varargin{:});
    dist = statslib.internal.checkArgsForFastEuc(dist,issparse(X),cacheSize,size(X,1),class(X));
end

% Integer/logical/char/anything data may be handled by a caller-defined
% distance function, otherwise it is converted to double.  Complex floating
% point data must also be handled by a caller-defined distance function.
if ~strcmp(dist,'usr')
    % Built-in distances do not work for string arrays, so convert those to
    % char and see if that will work.
    X = convertStringsToChars(X);

    if ~isfloat(X)
        warning(message('stats:pdist:DataConversion', class( X )));
        if iscellstr(X)
            X = char(X);
        end
        X = double(X);
    elseif any(imag(X(:)))
        error(message('stats:pdist:InvalidData'));
    end
end

if ~ismatrix(X)
    error(message('stats:pdist:UnsupportedND'));
end

[n,p] = size(X);

% Degenerate case, just return an empty of the proper size.
if n < 2
    if ~strcmp(dist,'usr')
        Y = zeros(1,0,class(X)); % X was single/double, or cast to double
    elseif isa(X,'single')
           Y = zeros(1,0,'single');
    else
           Y = zeros(1,0);
    end
    return;
end

switch dist
    case {'seu' 'fse'} % Standardized Euclidean weights by coordinate variance
        if isempty(additionalArg)
            additionalArg =  var(X,'omitmissing');
            if any(additionalArg == 0)
                warning(message('stats:pdist:ConstantColumns'));
            end
            additionalArg = 1./ additionalArg;
        else
            if ~(isvector(additionalArg) && length(additionalArg) == p...
                    && all(additionalArg >= 0))
                error(message('stats:pdist:InvalidWeights'));
            end
            if any(additionalArg == 0)
                  warning(message('stats:pdist:ZeroInverseWeights'));
            end
            %We will apply the inverse weight on each coordinate in the sum
            %of squares.
            additionalArg = 1./ (additionalArg .^2);
        end
        
    case 'mah' % Mahalanobis
        if isempty(additionalArg)
            additionalArg = cov(X,'omitrows');
           [T,flag] = chol(additionalArg);
        else
            if ~isequal(size(additionalArg),[p,p])
                error(message('stats:pdist:InvalidCov'));
            end
            %use cholcov because we also need to check whether the matrix is symmetric
            [T,flag] = cholcov(additionalArg,0);
        end
        if flag ~= 0
            error(message('stats:pdist:SingularCov'));
        end
        additionalArg = full(T \ eye(p));  %inv(T)   
       
    case 'min' % Minkowski distance needs a third argument
        if isempty(additionalArg)  % use default value for exponent
            additionalArg = 2;
        elseif ~( isscalar(additionalArg) && additionalArg > 0)
            error(message('stats:pdist:InvalidExponent'));
        end
    case 'cos' % Cosine
        [X,flag] = normalizeX(X);
        if flag
            warning(message('stats:pdist:ZeroPoints'));
        end
    case 'cor' % Correlation
        X = X - mean(X,2);
        [X, flag] = normalizeX(X);
        if flag
            warning(message('stats:pdist:ConstantPoints'));
        end
    case 'spe'  %Spearman
        X = tiedrank(X')'; % treat rows as a series
        X = X - (p+1)/2; % subtract off the (constant) mean
        [X,flag] = normalizeX(X);
        if flag
            warning(message('stats:pdist:TiedPoints'));
        end
end

% Note that if there is any code for case 'che','euc' or 'cit' in the
% above switch dist block, the some code need to be repeated in the
% corresponding block below.
if strcmp(dist,'min') % Minkowski distance
    if isinf(additionalArg) %the exponent is inf
        dist = 'che';
        additionalArg = [];
    elseif additionalArg == 2 %the exponent is 2
        dist = 'euc';
        additionalArg = [];
    elseif additionalArg == 1 %the exponent is 1
        dist = 'cit';
        additionalArg = [];
    end
end

% Call a mex file to compute distances for the standard distance measures
% and full real double or single data.
if ~strcmp(dist,'usr') && (isfloat(X) && ~issparse(X)) % ~usr => ~complex
    additionalArg = cast(additionalArg,class(X));
    if ~ismember(dist, {'feu' 'fse' 'fqe'})
        Y = internal.stats.pdistmex(X',dist,additionalArg);
    else
        % The number of elements that the cache can hold
        if isnumeric(cacheSize)
            numElemInCache = floor( cacheSize*1e6/bytePerElement(class(X)) );
        else % cacheSize = 'maximal'
            numElemInCache = size(X,1)^2;
        end

        if isempty(additionalArg)
            Xtranspose = X';
            xnorm = vecnorm(Xtranspose,2,1).^2;
            Y = internal.stats.pdistmex(Xtranspose,dist,additionalArg,xnorm,numElemInCache);
            clear Xtranspose;
        else
            % fast standard Euclidean distance
            % input data is A=X.*sqrt(additionalArg), so that A*A' == X*W*X', where W is diag(additionalArg)
            % This approach saves space to store W matrix
            xnorm = sum(X.^2.*additionalArg,2);
            Y = internal.stats.pdistmex((X.*sqrt(additionalArg))',dist,additionalArg,xnorm,numElemInCache);
        end
    end
elseif ~strcmp(dist,'usr') && isfloat(X) % ~usr => ~complex
    additionalArg = cast(additionalArg,class(X));
    Y = internal.stats.pdistSparseMEX(X', dist, additionalArg);
    
% This M equivalent assumes real single or double.  It is currently not
% called, but it may also be useful as a template for customization.
elseif ~strcmp(dist,'usr')
    if any(strcmpi(dist, {'ham' 'jac' 'che'}))
        nans = any(isnan(X),2);
    end
    outClass = class(X);
    Y = zeros(1,n*(n-1)./2, outClass);
    k = 1;
    for i = 1:n-1
        switch dist
            case {'euc','sqe'}    % Euclidean
                dsq = zeros(n-i,1,outClass);
                for q = 1:p
                    dsq = dsq + (X(i,q) - X((i+1):n,q)).^2;
                end
                if strcmp(dist,'euc')
                   Y(k:(k+n-i-1)) = sqrt(dsq);
                else
                   Y(k:(k+n-i-1)) = dsq;
                end
                
            case 'seu'    % Standardized Euclidean
                wgts = additionalArg;
                dsq = zeros(n-i,1,outClass);
                for q = 1:p
                    dsq = dsq + wgts(q) .* (X(i,q) - X((i+1):n,q)).^2;
                end
                Y(k:(k+n-i-1)) = sqrt(dsq);
                
            case 'cit'    % City Block
                d = zeros(n-i,1,outClass);
                for q = 1:p
                    d = d + abs(X(i,q) - X((i+1):n,q));
                end
                Y(k:(k+n-i-1)) = d;
                
            case 'mah'    % Mahalanobis
     
                 del = X(i,:) - X((i+1):n,:);
                 dsq = sum((del/T) .^ 2, 2);
                 Y(k:(k+n-i-1)) = sqrt(dsq);
            case 'min'    % Minkowski
                expon = additionalArg;
                dpow = zeros(n-i,1,outClass);
                for q = 1:p
                    dpow = dpow + abs(X(i,q) - X((i+1):n,q)).^expon;
                end
                Y(k:(k+n-i-1)) = dpow .^ (1./expon);
                
            case {'cos' 'cor' 'spe'}   % Cosine, Correlation, Rank Correlation
                % This assumes that data have been appropriately preprocessed
                d = zeros(n-i,1,outClass);
                for q = 1:p
                    d = d + (X(i,q).*X((i+1):n,q));
                end
                d(d>1) = 1; % protect against round-off, don't overwrite NaNs
                Y(k:(k+n-i-1)) = 1 - d;
                
            case 'ham'    % Hamming
                nesum = zeros(n-i,1,outClass);
                for q = 1:p
                    nesum = nesum + (X(i,q) ~= X((i+1):n,q));
                end
                nesum(nans(i) | nans((i+1):n)) = NaN;
                Y(k:(k+n-i-1)) = nesum ./ p;
                
            case 'jac'    % Jaccard
                nzsum = zeros(n-i,1,outClass);
                nesum = zeros(n-i,1,outClass);
                for q = 1:p
                    nz = (X(i,q) ~= 0 | X((i+1):n,q) ~= 0);
                    ne = (X(i,q) ~= X((i+1):n,q));
                    nzsum = nzsum + nz;
                    nesum = nesum + (nz & ne);
                end
                nesum(nans(i) | nans((i+1):n)) = NaN;
                Y(k:(k+n-i-1)) = nesum ./ nzsum;
                
            case 'che'    % Chebychev
                dmax = zeros(n-i,1,outClass);
                for q = 1:p
                    dmax = max(dmax, abs(X(i,q) - X((i+1):n,q)));
                end
                dmax(nans(i) | nans((i+1):n)) = NaN;
                Y(k:(k+n-i-1)) = dmax;
                
        end
        k = k + (n-i);
    end
    
    % Compute distances for a caller-defined distance function.
else % if strcmp(dist,'usr')
    try
        Y = feval(distfun,X(1,:),X(2,:),distargs{:})';
    catch ME
        if strcmp('MATLAB:UndefinedFunction', ME.identifier) ...
                && ~isempty(strfind(ME.message, func2str(distfun)))
            error(message('stats:pdist:DistanceFunctionNotFound', func2str( distfun )));
        end
        % Otherwise, let the catch block below generate the error message
        Y = [];
    end
    
    % Make the return have whichever numeric type the distance function
    % returns, or logical.
    if islogical(Y)
        Y = false(1,n*(n-1)./2);
    else % isnumeric
        Y = zeros(1,n*(n-1)./2, class(Y));
    end
    
    k = 1;
    for i = 1:n-1
        try
            Y(k:(k+n-i-1)) = feval(distfun,X(i,:),X((i+1):n,:),distargs{:})';
        catch ME
            if isa(distfun, 'inline')
                m = message('stats:pdist:InlineFunctionError');
                throw(addCause(MException(m.Identifier,'%s',getString(m)),ME));
            else
                m = message('stats:pdist:DistanceFunctionError',func2str(distfun));
                throw(addCause(MException(m.Identifier,'%s',getString(m)),ME));
            end
        end
        k = k + (n-i);
    end
end


%---------------------------------------------
% Normalize the rows in X to have unit norm.
function [X,flag] = normalizeX(X)
% Rescale each row by largest element to prevent over/underflow, and
% compute the 2-norm.
Xmax = max(abs(X),[],2);
X2 = X./Xmax;
Xnorm = sqrt(sum(X2.^2, 2));

% The norm will be NaN for rows that are all zeros, fix that for the test
% below.
Xnorm(Xmax==0) = 0;

% The norm will be NaN for rows of X that have any +/-Inf. Those should be
% Inf, but leave them as is so those rows will not affect the test below.
% The points can't be normalized, so any distances from them will be NaN
% anyway.

% Find points that are effectively zero relative to the point with largest norm.
flag = any(Xnorm <= eps(max(Xnorm)));

% Back out the rescaling, and normalize rows of X to have unit 2-norm.
% Rows can't be normalized become all NaN.
Xnorm = Xnorm .* Xmax;
X = X./Xnorm;

function bytePerElem = bytePerElement(outClass)
if strcmpi(outClass, 'double')
    bytePerElem = 8;
else
    bytePerElem = 4;
end
