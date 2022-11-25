% Script for generating elipses from drilling data.
% handle = figure(1);
dataName = '21.10.22-open-loop-drill.xlsx';
data = xlsread(dataName, 'wood');
% 3.6800	3.6200	3.6350	3.6500
% 3.8100	3.6800	3.7100	3.7700
% 3.6900	3.6800	3.6300	3.6100
% c1 - 6, c2 -7, c3 - 8, c4 - 9
hold on
colour = [1,0,0];

info = ones(size(data,1),1);
Legend = cell(size(data,1),2);
for i=1:size(data,1)-1
    c1 = data(i,6); 
    c2 = data(i,7);
    c3 = data(i,8);
    c4 = data(i,9);
    params = ellipse_approx_fit(c1,c2,c3,c4, colour);
    eccentricity = sqrt(params{1}.long_axis*params{1}.long_axis - params{1}.short_axis*params{1}.short_axis)/params{1}.long_axis;
    Legend{i,1} = params{2};
    Legend{i,2} = "Depth: " + data(i,3) + ", e: " + eccentricity;
    disp(Legend{i,2})
    colour(1) = rand(1);
    colour(2) = rand(1);
    colour(3) = rand(1);
%     legend(params{2}, Legend{i,2});
    axis equal
end

% We have run the algorithm from smallest to greatest depth from 5 to 20 in
% 5mm increments
charVector = [];

% legend({Legend{1,2},Legend{2,2},Legend{3,2},Legend{4,2}});

% charVector = {};
% 
% for i=1:size(info,2)
%     charVector(i) = cellstr(append("e: ", string(info(i))));
% end
% charVector = append("e: ", string(eccs(1)));
% legend(string(info));

hold off
    % Calculates ellipse eccentricity 
    % c1 - vertical measurement
    % c2 - horizontal measurement
    % c3 - diag with top right corner.
    % c4 - diag with top left corner.
function params = ellipse_approx_fit(c1,c2,c3,c4,colour)
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
    
    params = fit_ellipse(circle_x,circle_y,gca, colour);
%     disp(params{2})
end


function A = circleAtZero(r)
% Generate circle points
t = 0:pi/50:2*pi;
x = r*sin(t);
y = r*cos(t);

A(:,1) = x;
A(:,2) = y;
end

