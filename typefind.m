%Mechanism Type determination algorithm

function [typ] = typefind(fixed,crank1,coupler,crank2)
values=[fixed,crank1,coupler,crank2];
min=99999;
max=0;
for j=1:4 %min value is picked
    if(values(j)<=min)%min value is picked
        min=values(j);
        short=j;
    end
    if(values(j)>=max) %max value is picked
        max=values(j);
        long=j;
    end
end 
int1=0;
for j=1:4 %CHECK HERE LATER ON Fails at equal lengths
    if((((j~=long)&&(j~=short))||((values(j)==min)&&(j~=short))||(values(j)==max))&&(int1==1)) %This one is run after inter1 is placed
        inter2=j;
    end
    
    if((((j~=long)&&(j~=short))||((values(j)==min)&&(j~=short))||(values(j)==max)&&(j~=long))&&(int1==0))
        inter1=j;
        int1=1;
    end
end
        
        if((min+max)<(values(inter1)+values(inter2)))
             max; %First case
           if (short == 1)
            typ=1; %Double Crank
           elseif mod(short+2,4) == 1
            typ=0;%Double Rocker
           elseif ((mod(short+1,4)==1)||(mod(short-1,4)==1))
            typ=1;%Double 
           end
        end
        if((min+max)>(values(inter1)+values(inter2)))
        typ=0; %Double Rocker
        end
        if((min+max)==(values(inter1)+values(inter2))) 
             if short == 1
            typ=0; %Double Crank[CHANGE POINT]
             elseif mod(short+2,4) == 1
            typ=0; %'Double Rocker[CHANGE POINT]'
           elseif ((mod(short+1,4)==1)||(mod(short-1,4)==1))
            typ=0; %'Crank Rocker[CHANGE POINT]'
             end
        end
            
    

end