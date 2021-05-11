function q = inverseKinematics(cartesian_EE, q_0, step_size, tol)
%INVERSEKINEMATICS 
    q_k = q_0;
    delta = 100;
    while norm(delta) > tol
        J = getAnaliticJacobian(q_k);
        direct_kin = directKinematics(q_k);
        delta = step_size * J' *(cartesian_EE - direct_kin);
        q_k = q_k + delta;
    end
    q = q_k;
end

