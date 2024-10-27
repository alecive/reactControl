function [t, data, proc]=processData(path_prefix, dT)

    ipoptExitCode_col = 63;
    timeToSolve_s_col = 64;
    qIntegrated = 65:74;
    posEE = 8:10;
    targetPosEE = 11:13;
    oriEE = 17:19;
    targetOriEE = 20:22;
    qDot = 23:32;

    d=importdata([path_prefix 'reactCtrl/data.log']);

    % d = d(d(:,6) > 10,:);
    t = d(:,3)-d(1,3);
    data.time_diff = diff(t);
    data.time_diff(data.time_diff > 0.5) = nan;
    data.solver_time = d(:,timeToSolve_s_col);
    data.ipoptExit = d(:,ipoptExitCode_col);
    Hr = getRotMat(d(:, targetOriEE));
    He = getRotMat(d(:, oriEE));
    Hr(1:3,4,:)=permute(d(:, targetPosEE), [2,3,1]);
    He(1:3,4,:)=permute(d(:, posEE), [2,3,1]);
    H = pagemtimes(Hr, 'none', He, 'transpose');
    v = zeros(length(d),3);
    v(:,1) = H(3,2,:)-H(2,3,:);
    v(:,2) = H(1,3,:)-H(3,1,:);
    v(:,3) = H(2,1,:)-H(1,2,:);
    r = sqrt(sum(v.^2,2));
    d(1, 33:42)
    proc.posError = sqrt(sum((d(:, targetPosEE)-d(:, posEE)).^2,2));
    proc.theta = atan2(0.5*r,squeeze(0.5*(H(1,1,:)+H(3,3,:)+H(2,2,:)-1)));

    proc.pose_diff = zeros(length(d),1);
    for i = 1:length(d)
        proc.pose_diff(i) = norm(Hr(:,:,i)-He(:,:,i),'fro');
    end
    proc.cart_vels = diff(d(:,posEE))./dT;
    proc.cart_vels_norm = vecnorm(proc.cart_vels')';
    proc.cart_jerk = diff(diff(proc.cart_vels)./dT)./dT;
    proc.cart_jerk_norm = vecnorm(proc.cart_jerk')';
    proc.joint_vels = d(:,qDot);
    proc.joint_vels2 = diff(d(:,qIntegrated))./dT;
    proc.joint_vels_norm = vecnorm(d(:,qDot)')';
    proc.joint_vels_norm2 = vecnorm(proc.joint_vels2')';
    proc.joint_jerk = diff(diff(d(2:end,qDot))./dT)./dT;
    proc.joint_jerk2 = diff(diff(proc.joint_vels2)./dT)./dT;
    proc.joint_jerk_norm = vecnorm(proc.joint_jerk')';
    proc.joint_jerk_norm2 = vecnorm(proc.joint_jerk2')';
    proc.distance = sum(abs(diff(d(:,qIntegrated))));
end

function H = getRotMat(data)

    ang_mag = sqrt(sum(data.^2,2));
    ang_axis = data./ang_mag;

    H = repmat(eye(4,4), 1,1,length(data));
    c = cos(ang_mag);
    s = sin(ang_mag);
    C=1.0-c;

    xs =ang_axis(:,1).*s;
    ys =ang_axis(:,2).*s;
    zs =ang_axis(:,3).*s;
    xC =ang_axis(:,1).*C;
    yC =ang_axis(:,2).*C;
    zC =ang_axis(:,3).*C;
    xyC =ang_axis(:,1).*yC;
    yzC =ang_axis(:,2).*zC;
    zxC =ang_axis(:,3).*xC;
    
    H(1,1,:) = ang_axis(:,1).*xC + c;
    H(2,2,:) = ang_axis(:,2).*yC + c;
    H(3,3,:) = ang_axis(:,3).*zC + c;
    H(1,2,:)=xyC-zs;
    H(1,3,:)=zxC+ys;
    H(2,1,:)=xyC+zs;
    H(2,3,:)=yzC-xs;
    H(3,1,:)=zxC-ys;
    H(3,2,:)=yzC+xs;
end