function transform = imported_transform(data)
assert(isscalar(data),'Expected one imported transform for each endpoint.');
transform=eye(4);
if abs(data.angle)>1e-15
    axis=data.axis(:)/norm(data.axis);
    skew=[0 -axis(3) axis(2);axis(3) 0 -axis(1);-axis(2) axis(1) 0];
    transform(1:3,1:3)=eye(3)*cos(data.angle)+(1-cos(data.angle))*(axis*axis')+sin(data.angle)*skew;
end
transform(1:3,4)=data.translation(:);
end
