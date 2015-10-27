function f_dot_q = func_dot( q, qGuardMinExt, qGuardMinCOG, qGuardMinInt,...
                                qGuardMaxInt, qGuardMaxCOG, qGuardMaxExt,...
                                      qGuard,        W_min,      W_gamma )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    % f_dot_q=0.5*W_gamma*(1.0+tanh(-6.0*(q-qGuardMinCOG)/qGuard))+W_min;

    % f_dot_q=0.5*W_gamma*(1.0+tanh(6.0*(q-qGuardMaxCOG)/qGuard))+W_min; 

    disp('q')
    disp(q)

    if ((q>=qGuardMinInt) & (q<=qGuardMaxInt))
        disp('safe');
        f_dot_q=0.0;
    elseif ((q<=qGuardMinExt) | (q>=qGuardMaxExt))
        disp('outside');
        f_dot_q=0.0;
    elseif (q<qGuardMinInt)
        disp('inside min');
        f_dot_q= (3*W_gamma*(tanh((6*q - 6*qGuardMinCOG)/qGuard)^2 - 1))/qGuard;
    else
        disp('inside max');
        f_dot_q=-(3*W_gamma*(tanh((6*q - 6*qGuardMaxCOG)/qGuard)^2 - 1))/qGuard;
    end

end

% q=[-9:0.1:69];
% for(i=1:size(q,2)) disp(i); f_dot_q(i)=func_dot(q(i)); end
% plot(q,f_dot_q)
