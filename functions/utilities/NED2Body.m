function DCM = NED2Body(EulerAngles)

roll    = EulerAngles(:,1);
pitch   = EulerAngles(:,2);
yaw     = EulerAngles(:,3);

DCM     = zeros(3,3,length(roll));

for i = 1:length(roll)
    DCM(:,:,i) = [cos(yaw(i))*cos(pitch(i)), sin(yaw(i))*cos(pitch(i)), -sin(pitch(i));
    cos(yaw(i))*sin(pitch(i))*sin(roll(i)) - sin(yaw(i))*cos(roll(i)), ...
    sin(yaw(i))*sin(pitch(i))*sin(roll(i)) + cos(yaw(i))*cos(roll(i)), ...
    cos(pitch(i))*sin(roll(i));
    cos(yaw(i))*sin(pitch(i))*cos(roll(i)) + sin(yaw(i))*sin(roll(i)), ...
    sin(yaw(i))*sin(pitch(i))*cos(roll(i)) - cos(yaw(i))*sin(roll(i)), ...
    cos(pitch(i))*cos(roll(i))];
end

end