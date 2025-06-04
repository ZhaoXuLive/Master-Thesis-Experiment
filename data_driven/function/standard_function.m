function yout = standard_function(vars, polyorder)
% Copyright 2024, All Rights Reserved
% Code by Chao Liang
% For Paper, "xxxxxx"
% by Chao Liang

yn = size(vars, 1);
% yout = zeros(n,1+nVars+(nVars*(nVars+1)/2)+(nVars*(nVars+1)*(nVars+2)/(2*3))+11);

%% initial

% 1540
fxlen = 6+5+8;
uy = zeros(yn, fxlen);
uy(:, 1) = sin(vars(:, 1));
uy(:, 2) = sin(vars(:, 2));
uy(:, 3) = sin(vars(:, 3));
uy(:, 4) = cos(vars(:, 1));
uy(:, 5) = cos(vars(:, 2));
uy(:, 6) = cos(vars(:, 3));
uy(:, 6+1:6+5) = vars(:, 4:8);
uy(:, 6+5+1:end) = vars(:, 9:end);


%% add
ind = 1;
% poly order 0
yout(:,ind) = ones(yn,1);
ind = ind+1;

% poly order 1
for i=1:fxlen
    yout(:,ind) = uy(:,i);
    ind = ind+1;
end

if(polyorder>=2)
    % poly order 2
    for i=1:fxlen
        for j=i:fxlen
            yout(:,ind) = uy(:,i).*uy(:,j);
            ind = ind+1;
        end
    end
end

if(polyorder>=3)
    % poly order 3
    for i=1:fxlen
        for j=i:fxlen
            for k=j:fxlen
                yout(:,ind) = uy(:,i).*uy(:,j).*uy(:,k);
                ind = ind+1;
            end
        end
    end
end

if(polyorder>=4)
    % poly order 4
    for i=1:fxlen
        for j=i:fxlen
            for k=j:fxlen
                for l=k:fxlen
                    yout(:,ind) = uy(:,i).*uy(:,j).*uy(:,k).*uy(:,l);
                    ind = ind+1;
                end
            end
        end
    end
end

if(polyorder>=5)
    % poly order 5
    for i=1:fxlen
        for j=i:fxlen
            for k=j:fxlen
                for l=k:fxlen
                    for m=l:fxlen
                        yout(:,ind) = uy(:,i).*uy(:,j).*uy(:,k).*uy(:,l).*uy(:,m);
                        ind = ind+1;
                    end
                end
            end
        end
    end
end

end