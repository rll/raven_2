[N ~] = size(r);

E = zeros(N,16);
G = zeros(N,16);
for i=2:1:N
    p = reshape(c(i,:),4,4);
    q = reshape(r(i,:),4,4);
    residual = reshape(T*p - q,1,16); 
    E(i,:) = residual;
    G(i,:) = c(i,:) - c(i-1,:);
end

    plt = zeros(N,2);
    for i=1:1:N
        plt(i,:) = [norm(E(i,:)) norm(G(i,:))];
    end
