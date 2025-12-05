function Chs = RB_ObjectsFunc(pos,R,ChaserSide)


vertex_matrix_chs = [0 0 0; ChaserSide 0 0; ChaserSide ChaserSide 0; 0 ChaserSide 0; 0 0 ChaserSide; ChaserSide 0 ChaserSide; ChaserSide ChaserSide ChaserSide; 0 ChaserSide ChaserSide]-[ChaserSide ChaserSide ChaserSide]/2;
faces_matrix_chs = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];

colchs = ['red';'red';'red';'red';'red';'red'];


for kk = 1:length(vertex_matrix_chs)
    vertex_chs_CoM_ECI(kk,:) = pos+R*vertex_matrix_chs(kk,:)';
end 


Chs.Vertices = vertex_chs_CoM_ECI;
Chs.Faces = faces_matrix_chs;
Chs.FaceVertexCData = [0 1 0; 0 1 0;  0  1 0; 0 1 0; 0  1 0;  0 1 0];
Chs.FaceColor = 'flat';
Chs.EdgeColor = 'black';
Chs.LineWidth = 1;

