function [allAbsTrans, allAbsPoses] = accumulatedPoses(poseList)

allAbsTrans=zeros(3,3,size(poseList,1));
T=eye(3);
allAbsTrans(:,:,1)=T;
allAbsPoses=zeros(size(poseList,1),3);
for n=2:size(poseList,1)
    T=T*myTrans(poseList(n,:)); 
    allAbsTrans(:,:,n)=T;   
    tmpPose=[T(1,3), T(2,3), atan2(T(2,1),T(1,1)) ];
    allAbsPoses(n,:)=tmpPose;
end


function TT=myTrans(ppp)
x=ppp(1);
y=ppp(2);
a=ppp(3);

TT=[cos(a) -sin(a)  x
    sin(a)  cos(a)  y
    0        0      1];
