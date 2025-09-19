function animateFrame(quat,qaxis,axisOffset,bFrame)
numSteps = 50; % Number of animation frames
pauseTime = 0.05; % Pause duration per frame
% Animate rotation around the quaternion axis
ang=2*acos(quat.s);
allangs=linspace(0, ang, numSteps);
%Add some more frames :-)
allangs=[allangs repmat(allangs(end),1,10)...
fliplr(allangs(1:2:end))];
for m = allangs
% Compute incremental rotation matrix
% Convert axis-angle to 3x3 rotation matrix
R inc = axang2rotm([qaxis, m]);
% Convert to homogeneous 4x4
T inc = [R inc, [0; 0; 0]; 0, 0, 0, 1];
%Update base reference frame using the transform.
set(bFrame, 'Matrix', transl(axisOffset/2)*T inc);
pause(pauseTime); % Pause forsmooth animation
end
pause(1)
end
