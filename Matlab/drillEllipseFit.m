% Script for generating elipses from drilling data.
% handle = figure(1);
dataName = 'organised drill test data.xlsx';
data = xlsread(dataName);

% c1 - 6, c2 -7, c3 - 8, c4 - 9
hold on
colour = [1,0,0];
eccs = ones(1,size(data,1));
for i=1:size(data,1)
    c1 = data(i,6); 
    c2 = data(i,7);
    c3 = data(i,8);
    c4 = data(i,9);
    eccs(i) = ellipse_approx_fit(c1,c2,c3,c4, colour);
    colour(1) = rand(1);
    colour(2) = rand(1);
    colour(3) = rand(1);
    axis equal
end

charVector = {};

for i=1:size(eccs,2)
    charVector(i) = cellstr(append("e: ", string(eccs(i))));
end
% charVector = append("e: ", string(eccs(1)));
legend(string(eccs));

hold off
    % Calculates ellipse eccentricity 
    % c1 - vertical measurement
    % c2 - horizontal measurement
    % c3 - diag with top right corner.
    % c4 - diag with top left corner.
    function eccentricity = ellipse_approx_fit(c1,c2,c3,c4,colour)

        % Convert to points
        c1_1 = [0,c1/2];
        c1_2 = [0,-c1/2];

        c2_1 = [c2/2, 0];
        c2_2 = [-c2/2,0];

        c3_1 = [c3/2*cos(45),c3/2*sin(45)];
        c3_2 = [-c3/2*cos(45), -c3/2*sin(45)];

        c4_1 = [-c4/2*cos(45), c4/2*sin(45)];
        c4_2 = [c4/2*cos(45), -c4/2*sin(45)];

        circle_x = [c1_1(1), c1_2(1), c2_1(1),c2_2(1), c3_1(1),c3_2(1),c4_1(1),c4_2(1)];
        circle_y = [c1_1(2), c1_2(2), c2_1(2),c2_2(2), c3_1(2),c3_2(2),c4_1(2),c4_2(2)];
        
     
        handle = plot(0,0);

        params = fit_ellipse(circle_x,circle_y,gca, colour);
        eccentricity = sqrt(params.long_axis*params.long_axis - params.short_axis*params.short_axis)/params.long_axis;
        
%         c1d = circleAtZero(c1);
%         c2d = circleAtZero(c2);
%         c3d = circleAtZero(c3);
%         c4d = circleAtZero(c4);


    end


function A = circleAtZero(r)
% Generate circle points
t = 0:pi/50:2*pi;
x = r*sin(t);
y = r*cos(t);

A(:,1) = x;
A(:,2) = y;
end

