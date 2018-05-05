function [qt, vt, at] = trapezoidal(qi, qf, vc, tf, t)
    %qi - initial position
    %qf - final position
    %tf - final time
    %vc - cruise velocity
    %t - current time instant
    
    n = size(qi, 1);
    qt = zeros(n, 1);
    vt = zeros(n, 1);
    at = zeros(n, 1);
    
    for i = 1:n
        delta = qf(i) - qi(i);
        
        if (delta==0)
            qt(i) = qi(i);
            vt(i) = 0;
            at(i) = 0;
        else
            vc_i = sign(delta) * abs(vc(i));
            %cruise time
            tc = (-delta + vc_i*tf)/vc_i;
            %constant acceleration
            ac_i = vc_i / tc;
            
            
            if (t >= 0) && (t <= tc)
                qt(i) = qi(i) + 0.5 * ac_i * t*t;
                vt(i) = ac_i *t;
                at(i) = ac_i;
            elseif (t > tc) && (t <= (tf - tc))
                qt(i) = qi(i) + ac_i * tc*(t - 0.5*tc);
                vt(i) = ac_i*tc;
                at(i) = 0;
            elseif (t > (tf - tc)) && (t <= tf)
                qt(i) = qf(i) - 0.5 * ac_i * (tf - t)*(tf - t);
                vt(i) = ac_i *(tf - t);
                at(i) = -ac_i;
            else
                qt(i) = qf(i);
                vt(i) = 0;
                at(i) = 0;
            end
        end
    end 
end