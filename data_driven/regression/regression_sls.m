function Xi = regression_sls(Theta, dXdt, lambda)

% compute Sparse regression: sequential least squares

rightDXDT = dXdt(:, 6:10);

Xi = Theta\rightDXDT;
% Xi = lsqminnorm(Theta, rightDXDT);
for k=1:10
    smallinds = (abs(Xi)<lambda);     % find small coefficients
    Xi(smallinds) = 0;                % and threshold
    for ind = 1:5                     % n is state dimension
        biginds = ~smallinds(:,ind);
        Xi(biginds,ind) = Theta(:,biginds)\rightDXDT(:,ind);
%         Xi(biginds,ind) = lsqminnorm(Theta(:,biginds), rightDXDT(:,ind)); 
    end
end
