    
%Iterative 3 point dyad equation for any curve desired
%METHOD1
tic;
clc;
clear;
curve1x=linspace(-51.04*sind(30),51.04*sind(30),100);
curve1y=linspace(-51.04*cosd(30),-51.04*cosd(30),100);
theta=linspace(-30,30,100);

curve2x=51*sind(theta);
curve2y=-51*cosd(theta);

figure(1)%Desired Movement
clf;
plot(curve1x,curve1y);

hold on;
scatter(0,0)
plot(curve2x,curve2y);
axis([-50 50 -60 0])
drawnow;

%Now randomly pick points on the curves and try to find a solution

done=0;
count=1;
%Generation part
while done==0
    b2a=degtorad(180*rand); %Randomize inputs commence
b3a=b2a+degtorad(round(180*rand));
b2b=degtorad(180*rand);
b3b=b2b+degtorad(round(180*rand));
if(abs((b2b-b2a))<5)
    b2b=b2b+10*rand;
end
if(abs((b3b-b3a))<5)
    b3b=b3b+pi*rand;
end                          %Randomize inputs end
    
    
    
p1=round(33*rand); 
if(p1==0)
    p1=1;
end
p2=34+round(33*rand); %Up to 67
p3=68+round(32*rand);
point1=curve2x(p1)+curve2y(p1)*1i; %These random numbers do change every loop
point2=curve2x(p2)+curve2y(p2)*1i;
point3=curve2x(p3)+curve2y(p3)*1i;
s2= point2-point1;
s3= point3-point1;
alph2=degtorad(round(180*rand));    %Alpha values are randomized 
                  %YOU ARE CONSTRAINING FREE PARAMETERS HERE!
alph3=alph2+degtorad((180-alph2)*rand); 



Aa=[(exp(b2a*i)-1),(exp(alph2*i)-1);(exp(b3a*i)-1),(exp(alph3*i)-1)];%For first Crank 
Ba=[s2;s3];
ka=linsolve(Aa,Ba);
wa=ka(1);
Wa=[abs(wa),mod(radtodeg(angle(wa)),360)];
za=ka(2);
Za=[abs(za),mod(radtodeg(angle(za)),360)];
magza(count)=abs(za);

Ab=[(exp(b2b*i)-1),(exp(alph2*i)-1);(exp(b3b*i)-1),(exp(alph3*i)-1)]; %for second crank-follower
Bb=[s2;s3];
kb=linsolve(Ab,Bb);
wb=kb(1);
Wb=[abs(wb),mod(radtodeg(angle(wb)),360)];
zb=kb(2);
Zb=[abs(zb),mod(radtodeg(angle(zb)),360)];
magzb(count)=abs(zb);
itwork=1;
thinitial(count)=Wa(2);
phinitial(count)=Wb(2);
a1(count)=abs((point1-za-wa)-(point1-zb-wb));%correct
a2(count)=Wa(1);                               %CHECKING THE ONLY CASES FOR WA CRANK CASE
a3(count)=abs((point1-za)-(point1-zb));%correct
a4(count)=Wb(1);
A0(count)=point1-za-wa;
B0(count)=point1-zb-wb;

if((a1(count)>5)&&(a2(count)>5)&&(a3(count)>5)&&(a4(count)>5))
type=typefind(a1(count),a2(count),a3(count),a4(count)); %Returns 1 if a2 is fully rotatable otherwise 0
else
  type=0; %SINGULAR MATRIX CASE
end
if((imag(A0(count))>0)&&(imag(B0(count))>0)&&(type==1)) %Ensures fixed pivots are on the robot and fully rotatable
%     disp('Check 1 complete');
    th1=mod(radtodeg(angle(B0(count)-A0(count))),360); %Branch control Angle Conversions
    th12=mod(Wa(2)-th1,360);
    th14=mod(Wb(2)-th1,360);
    step=1000;
    thinput=linspace(0,360,step); 
    [existence,phi1(count,:),phi2(count,:)]=fourbarsolve(a1(count),a2(count),a3(count),a4(count),thinput,step);
  
   if((5<a2(count))&&(a2(count)<40)&&(5<a4(count))&&(a4(count)<40)&&(abs(A0(count))<100)&&(abs(B0(count))<100)&&(abs(za)<70)&&(abs(zb)<70)&&(existence==1)) %LENGTH REQUIREMENT + MECHANISM BEING PHYSICAL +Fixed pivots being not so far
   
   th131=mod(radtodeg(atan2(a4(count)*sind(phi1(count,:))-a2(count)*sind(thinput),a1(count)+a4(count)*cosd(phi1(count,:))-a2(count)*cosd(thinput))),360);
   th132=mod(radtodeg(atan2(a4(count)*sind(phi2(count,:))-a2(count)*sind(thinput),a1(count)+a4(count)*cosd(phi2(count,:))-a2(count)*cosd(thinput))),360);
   th31=mod(th131+th1,360);
   th32=mod(th132+th1,360);
   
   gama(count)=radtodeg(angle(za-zb)-angle(za));%Correct!
   couplerp1(count,:)=A0(count)+a2(count)*exp(degtorad(thinput+th1)*1i)+Za(1)*exp(degtorad(th31-gama(count))*1i);% Plus config
   couplerp2(count,:)=A0(count)+a2(count)*exp(degtorad(thinput+th1)*1i)+Za(1)*exp(degtorad(th32-gama(count))*1i);% Minus config
   str=['a2 is ',num2str(a2(count)),' a4 is ',num2str(a4(count)),' Gama is ',num2str(gama(count))];
   disp(str);
   figure(count)
   toc
    clf;
plot(curve1x,curve1y,'b');
hold on;
plot(curve2x,curve2y,'b');
z1x=[curve2x(p1) curve2x(p1)-real(za)];
z1y=[curve2y(p1) curve2y(p1)-imag(za)];
z2x=[curve2x(p1) curve2x(p1)-real(zb)];
z2y=[curve2y(p1) curve2y(p1)-imag(zb)];
w1x=z1x-[real(za) real(wa)];
w1y=z1y-[imag(za) imag(wa)];
w2x=z2x-[real(zb) real(wb)];
w2y=z2y-[imag(zb) imag(wb)];
patch(w1x,w1y,'r'); %Crank
patch(z1x,z1y,'r'); %Crank to point
patch(w2x,w2y,'b'); %Follower 
patch(z2x,z2y,'b'); %Follower to point
scatter(w1x(2),w1y(2),'b'); %Fixed joint place
scatter(w2x(2),w2y(2),'r'); %Places of joint place
scatter(z1x(2),z1y(2),'b'); %Places of fixed joints
scatter(z2x(2),z2y(2),'r'); %Places of fixed joints
scatter(real(point1),imag(point1),'m');
scatter(real(point2),imag(point2),'m');
scatter(real(point3),imag(point3),'m');
plot(real(couplerp1(count,:)),imag(couplerp1(count,:)),'c'); %Plus config
plot(real(couplerp2(count,:)),imag(couplerp2(count,:)),'m'); %Minus config
strfinal=['Mechanism :',num2str(count)];
title(strfinal);
drawnow;
   count=count+1;
   
   end
end



if(count==501) %N many mechanisms are generated.(Input N+1 to count==)
   done=1;
   
end

end
toc

load gong.mat;  %To show the analysis is complete a loud sound is played
sound(y);
