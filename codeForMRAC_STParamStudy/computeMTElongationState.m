function [x1, x1dot, L_vecs, L_mags ,E_vecs ,VR_vecs] = computeMTElongationState(state_k, idx_nodes_start, N_mt_nodes, chaserSideLength, l0vec)
    % Compute MT elongation state from the current simulated state vector
    % using the same segment-geometry logic as the dynamics function.

    posChaser = state_k(1:3);
    velChaser = state_k(4:6);
    q_C = state_k(7:10);
    omegaChaser = state_k(11:13);

    nodes_pos = zeros(3, N_mt_nodes);
    nodes_vel = zeros(3, N_mt_nodes);
    for i = 1:N_mt_nodes
        idx = idx_nodes_start + (i-1)*6;
        nodes_pos(:, i) = state_k(idx:idx+2)';
        nodes_vel(:, i) = state_k(idx+3:idx+5)';
    end

    rotMat_C_A_I = [ ...
        q_C(1)^2+q_C(2)^2-q_C(3)^2-q_C(4)^2, 2*(q_C(2)*q_C(3)-q_C(1)*q_C(4)), 2*(q_C(2)*q_C(4)+q_C(1)*q_C(3));
        2*(q_C(2)*q_C(3)+q_C(1)*q_C(4)), q_C(1)^2-q_C(2)^2+q_C(3)^2-q_C(4)^2, 2*(q_C(4)*q_C(3)-q_C(1)*q_C(2));
        2*(q_C(2)*q_C(4)-q_C(3)*q_C(1)), 2*(q_C(3)*q_C(4)+q_C(1)*q_C(2)), q_C(1)^2-q_C(2)^2-q_C(3)^2+q_C(4)^2]';

    distAttPt_to_C = 0.5*chaserSideLength*[0 0 -1]';
    pos_Att = posChaser + rotMat_C_A_I' * (distAttPt_to_C);
    vel_Att = velChaser + rotMat_C_A_I' * (cross(omegaChaser, distAttPt_to_C));

        % Prep Pos/Vel Matrices for Predecessor (Prev) and Current (Curr) points
    % Prev: [pos_Att, Node 1, Node 2 ... Node N-1]
    P_prev = [pos_Att, nodes_pos(:, 1:end-1)];
    V_prev = [vel_Att, nodes_vel(:, 1:end-1)];

    % Curr: [Node 1, Node 2 ... Node N]
    P_curr = nodes_pos;
    V_curr = nodes_vel;
    
    % Vectorized Force Calc
    L_vecs = P_curr - P_prev;
    L_mags = sqrt(sum(L_vecs.^2, 1));
    E_vecs = L_vecs ./ L_mags;
    VR_vecs = V_curr - V_prev;

    x1 = sum(L_mags) - l0vec(1)*N_mt_nodes;
    x1dot = 0;
    for kk = 1:N_mt_nodes
        x1dot = x1dot + dot(VR_vecs(:, kk), E_vecs(:, kk));
    end
end
