%% Setup reading from directory
% d = uigetfile('*.csv', 'Select Multiple Files', 'MultiSelect', 'on');
% disp(d)

%% Get files
List = dir('B:\Uni\Thesis\Thesis_robotic_drilling\Matlab\Data\21-10-22\*joint_states.xlsx');
FileNames = fullfile({List.folder}, {List.name});

%% Iterate over files generating cartesian position
for j=1:numel(FileNames)
%% Import the data and analyze
    disp("Analysing data");
    jointstates = readtable(cell2mat(FileNames(j)));
    arrayJointStates = table2array(jointstates(:,10));
    cartPosVector = zeros(length(arrayJointStates),4);

    for i=1:size(jointstates(:,10))
        % Grab joint position from 10th column
        jointpos = arrayJointStates(i);
        jointpos = str2num(cell2mat(jointpos));
        cartPos = applyTransformation(jointpos);
        cartPosVector(i,:) = [cartPos(1,4), cartPos(2,4), cartPos(3,4), i];
    end
    
    % Choose data to save. X,Y,Z
    % disp(cartPosVector);
    % Concatenate with timestamp column
    finalMatrix = cell(size(cartPosVector,1), size(cartPosVector,2)+1);
    finalMatrix(:,1) = table2cell(jointstates(:,1)); 
    finalMatrix(:,2:5) = num2cell(cartPosVector);
    outputPath = "CartesianPosition/" + List(j).name;
    writetable(cell2table(finalMatrix), outputPath);
    % Open csv
end

%% Magic function
function cartesianPos = applyTransformation(jointpos)
    rectJointPos = [jointpos(3),jointpos(2),jointpos(1),jointpos(4),jointpos(5), jointpos(6)];

    dhParams = [
%         theta, a, d, alphadh
           rectJointPos(1),     0,   0.1625,  pi/2;
           rectJointPos(2), -0.4250,    0,    0 ;
           rectJointPos(3), -0.3922,    0,    0 ;
           rectJointPos(4),     0  , 0.1333,  pi/2;
           rectJointPos(5),     0  , 0.0997, -pi/2;
           rectJointPos(6),     0  , 0.0996,  0;
    ];

%     interimResult;

    
 
    interimResult = eye(4,4);
    for i=1:size(jointpos,2)
       theta = dhParams(i,1);
       a = dhParams(i,2);
       d = dhParams(i,3);
       alphadh = dhParams(i,4);
       
       tfMatrix = [
        cos(theta), -sin(theta)*cos(alphadh), sin(theta)*sin(alphadh) , a*cos(theta);
        sin(theta), cos(theta)*cos(alphadh) , -cos(theta)*sin(alphadh), a*sin(theta);
        0         , sin(alphadh)            , cos(alphadh)            , d           ;
        0         , 0                       , 0                       , 1           ;
    
       ];
     
       interimResult = interimResult * tfMatrix;
       
    end
    
    
    cartesianPos = interimResult;
    
   
end







