function f= TestFunction(x)

%%
% 1-d
%f =((x*6-2).^2).*sin((x*6-2).*2);

%%
% Branin function 
% The number of variables n = 2. 
% Variable range: -5<=x1<=10 , 0<=x2<=15
% x* =  (-pi, 12.275), (pi , 2.275), (9.42478, 2.475), 
% f(x*) = 0.397887
% 
% f = (x(2)-(5.1/(4*pi^2))*x(1)^2+5*x(1)/pi-6)^2+10*(1-1/(8*pi))*cos(x(1))+10;
%
 
%%
% % Michalewicz function 
% n = 2; 
% m = 10;
%  f = 0;
%  for i = 1:n;
%      f = f+sin(x(i))*(sin(i*x(i)^2/pi))^(2*m);
%  end
%  f = -f;

%%
% % % Shubert function
% s1 = 0; 
% s2 = 0;
% for i = 1:5;   
%     s1 = s1+i*cos((i+1)*x(1)+i);
%     s2 = s2+i*cos((i+1)*x(2)+i);
% end
% f = s1*s2;

%%
% % Goldstein and Price function 
% % -2 < xi < 2
% a = 1+(x(1)+x(2)+1)^2*(19-14*x(1)+3*x(1)^2-14*x(2)+6*x(1)*x(2)+3*x(2)^2);
% b = 30+(2*x(1)-3*x(2))^2*(18-32*x(1)+12*x(1)^2+48*x(2)-36*x(1)*x(2)+27*x(2)^2);
% f = a*b;

%%
% Dixon and Price function.
% n = 2;
% s1 = 0;
% for j = 2:n;
%     s1 = s1+j*(2*x(j)^2-x(j-1))^2;    
% end
% f = s1+(x(1)-1)^2;

%%

f = (1-x(1))^2 + 100*(x(2) - x(1)^2)^2;
end