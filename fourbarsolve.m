    
function [existence,phicalc1,phicalc2]=fourbarsolve(a1,a2,a3,a4,th12,step)


A=a1^2+a2^2-a3^2+a4^2-2*a1*a2*cos(degtorad(th12));
B=2*a1*a4-2*a2*a4*cos(degtorad(th12));
C=-2*a2*a4*sin(degtorad(th12));
existence=1;    
for m=1:step
checker=C(m)^2+B(m)^2-A(m)^2;
if(checker<0) %Mechanism does not exist
  
   existence=0;
   phicalc1=linspace(0,0,step);
   phicalc2=linspace(0,0,step);
   return;
end
end
if(existence==1)
phicalc1=mod(radtodeg(2*atan2(-C+sqrt(C.^2+B.^2-A.^2),A-B)),360); % 1st configuration
phicalc2=mod(radtodeg(2*atan2(-C-sqrt(C.^2+B.^2-A.^2),A-B)),360); %2nd configuration




end





