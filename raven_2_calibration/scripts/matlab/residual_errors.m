[N ~] = size(r);

E = zeros(N,16);
T = [R t; zeros(1,3) 1];
for i=1:1:N
    p = reshape(c(i,:),4,4)';
    q = reshape(r(i,:),4,4)';
    residual = reshape(T*p - q,1,16);
    E(i,:) = residual;
end

    subplot(3,3,1);
    hold on;
    histfit(E(:,1),50)
    xlim([-.35 .35])
    
        subplot(3,3,2);
    hold on;
    histfit(E(:,2),50)
    xlim([-.35 .35])
    
        subplot(3,3,3);
    hold on;
    histfit(E(:,3),50)
    xlim([-.35 .35])
    
       subplot(3,3,4);
    hold on;
    histfit(E(:,5),50)
    xlim([-.35 .35])
    
        subplot(3,3,5);
    hold on;
    histfit(E(:,6),50)
    xlim([-.35 .35])
    
        subplot(3,3,6);
    hold on;
    histfit(E(:,7),50)
    xlim([-.35 .35])
    
        subplot(3,3,7);
    hold on;
    histfit(E(:,9),50)
    xlim([-.35 .35])
    
        subplot(3,3,8);
    hold on;
    histfit(E(:,10),50)
    xlim([-.35 .35])
    
        subplot(3,3,9);
    hold on;
    histfit(E(:,11),50)
    xlim([-.35 .35])
