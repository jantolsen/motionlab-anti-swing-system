function plot_line(p1, p2)
%         Pre-allocate
        line = zeros(2,2);
        line(1,:) = p1;
        line(2,:) = p2;

        plot(line(:,1), line(:,2), 'k')
    end