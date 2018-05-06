function out = get_qvrep(q)
% GET_QVREP transforms input joint space coordinates q into V-REP joint 
% space coordinates for Kuka robot

    out = q;

    out(2) = -q(2);
    out(6) = -q(6);
end

