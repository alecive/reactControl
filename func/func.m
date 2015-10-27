function f_q = func( q, qGuardMinExt, qGuardMinCOG, qGuardMinInt,...
                        qGuardMaxInt, qGuardMaxCOG, qGuardMaxExt,...
                              qGuard,        W_min,      W_gamma )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    % f_q=0.5*W_gamma*(1.0+tanh(-6.0*(q-qGuardMinCOG)/qGuard))+W_min;

    % f_q=0.5*W_gamma*(1.0+tanh(6.0*(q-qGuardMaxCOG)/qGuard))+W_min; 

    % disp('q')
    disp(q)

    if ((q>=qGuardMinInt) & (q<=qGuardMaxInt))
        disp('safe');
        f_q=W_min;
    elseif ((q<=qGuardMinExt) | (q>=qGuardMaxExt))
        disp('outside');
        f_q=W_min+W_gamma;
    elseif (q<qGuardMinInt)
        disp('inside min');
        f_q=0.5*W_gamma*(1.0+tanh(-6.0*(q-qGuardMinCOG)/qGuard))+W_min;
    else
        disp('inside max');
        f_q=0.5*W_gamma*(1.0+tanh( 6.0*(q-qGuardMaxCOG)/qGuard))+W_min;
    end

end

% q=[-9:0.1:69];
% for(i=1:size(q,2)) disp(i); f_q(i)=func(q(i)); end
% plot(q,f_q)
