function [Align, TethVec_CtoT] = AlignmentCalculator(Att_C,Att_T,rc,rt,qc,qt)

cReci =  RotMat(qc,1);   
tReci =  RotMat(qt,1);

rc_att = rc'+cReci'*Att_C;
rt_att = rt'+tReci'*Att_T;

TethVec_CtoT = -(rc_att-rt_att)/norm(rc_att-rt_att);
TargY_neg = tReci'*Att_T./norm(tReci'*Att_T);

Align = acosd(dot(-TethVec_CtoT,TargY_neg)/(norm(TargY_neg)*norm(TethVec_CtoT)));

end

