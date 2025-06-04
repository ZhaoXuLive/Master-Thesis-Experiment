function yout = sine_function_im(yin, u, trq, polyorder)
% Copyright 2024, All Rights Reserved
% Code by Chao Liang
% For Paper, "xxxxxx"
% by Chao Liang

yn = size(yin,1);

%% initial

% 把y和u综合放下一起
fxlen = 8+8+8;
uy = zeros(yn, fxlen);
uy(:, 1:8) = yin(:, 3:10);
uy(:, 9:16) = u;
uy(:, 17:end) = trq;

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

for k = 1:1
    yout = [yout sin(k*uy) cos(k*uy)];
end

end