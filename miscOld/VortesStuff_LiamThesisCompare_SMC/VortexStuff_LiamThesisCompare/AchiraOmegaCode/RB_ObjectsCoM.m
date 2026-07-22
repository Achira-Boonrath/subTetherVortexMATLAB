
vertex_matrix_chs = [0 0 0; 1 0 0; 1 1 0; 0 1 0; 0 0 1; 1 0 1; 1 1 1; 0 1 1]-[1 1 1]/2;
faces_matrix_chs = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];

vertex_matrix_tgt = [0 0 0; 1.25 0 0; 1.25 1.75 0; 0 1.75 0; 0 0 1.25; 1.25 0 1.25; 1.25 1.75 1.25; 0 1.75 1.25]-[1.25 1.75 1.25]/2;
faces_matrix_tgt = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];

coltgt = ['red';'red';'red';'red';'red';'red'];
colchs = ['blue';'blue';'blue';'blue';'blue';'blue'];


for kk = 1:length(vertex_matrix_chs)
    
    vertex_chs_CoM_ECI(kk,:) = pos0(jj,1:3)'+chsb_R_eci0'*vertex_matrix_chs(kk,:)';
    vertex_tgt_CoM_ECI(kk,:) = pos0(jj,end-2:end)'+targb_R_eci0'*vertex_matrix_tgt(kk,:)';
    
    vertex_chs_CoM_LVLH(jj).M(kk,:) = LVLH_R_ECI*(vertex_chs_CoM_ECI(kk,:)'-CenterOfMassPosition(jj,1:3)');
    vertex_tgt_CoM_LVLH(jj).M(kk,:) = LVLH_R_ECI*(vertex_tgt_CoM_ECI(kk,:)'-CenterOfMassPosition(jj,1:3)');
    
    
end 
%vertex_tgt_CoM_LVLH(jj).M
% THIS ONE FOR FINDING CHANGES 
% vertex_tgt_CoM_LVLH(jj).M = vertex_tgt_CoM_LVLH(jj).M-([ 0.464498331698706 30.496427383029371-30.5 0])
Targ(jj).Vertices = vertex_tgt_CoM_LVLH(jj).M;
Targ(jj).Faces = faces_matrix_tgt;
FV1 = [0 0 1];
Targ(jj).FaceVertexCData = [FV1;FV1;FV1;FV1;FV1;FV1];
Targ(jj).FaceColor = 'flat';
Targ(jj).EdgeColor = 'black';
Targ(jj).LineWidth = 1;

Chs(jj).Vertices = vertex_chs_CoM_LVLH(jj).M;
Chs(jj).Faces = faces_matrix_chs;
Chs(jj).FaceVertexCData = [0 1 0; 0 1 0;  0  1 0; 0 1 0; 0  1 0;  0 1 0];
Chs(jj).FaceColor = 'flat';
Chs(jj).EdgeColor = 'black';
Chs(jj).LineWidth = 1;

