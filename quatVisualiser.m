IMUdata = table2array(IMU);
dims = size(IMUdata);

% Graph the original vector.
% quatPlot = quiver3(0,0,0,0,0,1);
initVector = [0;0;0;1];
h = plot3(0,0,0);
% Use hamiltonian product
% P' = RPR'
currQuat = [];
invCurrQuat = [];
currVector = [];
oldVector = initVector;

for i=1:dims(1)
   pause(0.01) 
   currQuat = [IMUdata(i,1), IMUdata(i,2), IMUdata(i,3), IMUdata(i,4)];
   invCurrQuat = -1.*currQuat;
   invCurrQuat(1,1) = currQuat(1,1);
   % We now have a rotated current vector
   currVector = currQuat*oldVector*invCurrQuat';
   % Make a line of points between it and 0,0,0
   x = linspace(currVector(1),0);
   y = linspace(currVector(2),0);
   z = linspace(currVector(3),0);
   set(h, 'xdata', x, 'ydata', y, 'zdata', z)
   
   
   
   
   oldVector = currVector;
end 