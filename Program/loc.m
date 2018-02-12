clear;
clc;

range = [12000 9000 7500];
%parameter 12000 9000 7500 need to be adjust for the max and second max
%distance value
load('C.mat');

figure(1);
hold on;
axis([1 1024 1 768]);
x_star = [0, 0, -11.655, 10.563] / 29 * 101.7 + 1024 / 2;                   % to adjust the data for the scale between the pixel and the real point
y_star = [14.5, -14.5, 8.741, 2.483] / 29 * 101.7 + 768 / 2;
plot(x_star , y_star, '*');

for i = 1:size(rawStarData, 1)
    
    x = rawStarData(i, 1:4);
    y = rawStarData(i, 5:8);
    for j = 4:-1:1                                                          % to read reliable data
        if (x(j)==1023 || y(j)==1023)
            x(j) = [];
            y(j) = [];
        end
    end
    
    if length(x) < 3                                                        % only work with more than 3 points location readable
        disp('less than 3 points');
        disp(i);
    else
        dis = zeros(length(x)* (length(x) - 1) / 2, 3);                     %calculate the number of distance
        l = 1;
        for j = 1: (length(x) -1)
            for k = (j+1): length(x)
                dis(l, :) = [j, k, ((x(j) - x(k))^2 + (y(j) - y(k))^2)];    %save point and distance
                for m = l:-1:2                                              %bubble sort high to low
                    if dis(m, 3) > dis(m - 1, 3)
                        c = dis(m - 1, :);
                        dis(m - 1, :) = dis(m, :);
                        dis(m, :) = c;
                    end
                end
                
                l = l + 1;
            end
        end

        if dis(1, 3) > range(1)                                             %12000 for the range of max distance                            
            continue;
        else if dis(1, 3) > range(2)                                        %case 1 and 2 both exist
                if dis(1, 1) == dis(2, 1)                                   %9000 for the range of max distance
                    x_max1 = x(dis(1, 1));
                    x_max2 = x(dis(1, 2));
                    y_max1 = y(dis(1, 1));
                    y_max2 = y(dis(1, 2));
                else
                    x_max1 = x(dis(1, 2));
                    x_max2 = x(dis(1, 1));
                    y_max1 = y(dis(1, 2));
                    y_max2 = y(dis(1, 1));
                end
             
            else                                                            %set point 3 as the left, 4 as the right
                                                                            %case only 1\3\4 exist
                if dis(1, 3) > range(3)                                     %7500 for the range of second max distance
                    x_max1 = x(dis(1, 1) + dis(1, 2) + dis(2, 1) + dis(2, 2) - 6);
                    y_max1 = y(dis(1, 1) + dis(1, 2) + dis(2, 1) + dis(2, 2) - 6);
                    ind_4 = dis(1, 1) + dis(1, 2) + dis(3, 1) + dis(3, 2) - 6;
                    ind_3 = dis(2, 1) + dis(2, 2) + dis(3, 1) + dis(3, 2) - 6;
                    x_max2 = x_max1 + 3.2073 * (0.4754 * (x(ind_4) - x_max1) + 0.5246 * (x(ind_3) - x_max1));
                    y_max2 = y_max1 + 3.2073 * (0.4754 * (y(ind_4) - y_max1) + 0.5246 * (y(ind_3) - y_max1));
                else                                                        %case only 2\3\4 exist
                    x_max2 = x(dis(2, 1) + dis(2, 2) + dis(3, 1) + dis(3, 2) - 6);
                    y_max2 = y(dis(2, 1) + dis(2, 2) + dis(3, 1) + dis(3, 2) - 6);
                    ind_3 = dis(1, 1) + dis(1, 2) + dis(2, 1) + dis(2, 2) - 6;
                    ind_4 = dis(1, 1) + dis(1, 2) + dis(3, 1) + dis(3, 2) - 6;
                    x_max1 = x_max2 + 1.4530 * (0.4754 * (x(ind_4) - x_max2) + 0.5246 * (x(ind_3) - x_max2));
                    y_max1 = y_max2 + 1.4530 * (0.4754 * (y(ind_4) - y_max2) + 0.5246 * (y(ind_3) - y_max2));
                end
                
            end
        end

        theta1 = atan2((y_max2 - y_max1), (x_max2 - x_max1));               %theta1 for the vector of iWii
        theta2 = atan2((768 - y_max1 - y_max2), (1024 - x_max1 - x_max2));  %theta2 for the vector 1->2
        theta = theta2 - theta1;                                            %theta for the vector of iWii in global coordination                     
        len = sqrt((1024 - x_max1 - x_max2)^2 + (768 - y_max1 - y_max2)^2)/2;%distance from origin
        x_robot = -sin(theta) * len + 1024/2;                               %relocate iWii in global coordination 
        y_robot = cos(theta) * len + 768/2;

        plot(x_robot, y_robot, 'or');
        quiver(x_robot, y_robot, 100 * sin(theta1), 100 * cos(theta1), 'b');
    end
    pause(0.001);
end

figure(2);
hold on;
axis([1 1024 1 768]);
for i = 1:size(rawStarData, 1)
    plot(rawStarData(i,1),rawStarData(i,5),'r*')
    plot(rawStarData(i,2),rawStarData(i,6),'g*')
    plot(rawStarData(i,3),rawStarData(i,7),'b*')
    plot(rawStarData(i,4),rawStarData(i,8),'y*')
    pause(0.001)
end

