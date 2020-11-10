function   [ARG]=AngleUnwrap(A,B,ind)
% Placing in right quadrant
if B<0 && A>0 
ARG=abs(atan(A/B))+pi/2;
elseif B<0 && A<0
 ARG=abs(atan(A/B))+pi;
 
elseif B>0 && A<0
   
ARG=abs(atan(A/B))+3*pi/2;
else
    ARG=atan(A/B);
end
% Unwrapping part
if ind==-1 
    ARG=ARG-2*pi;
else if ind==1;
        
            ARG=ARG+2*pi;
    end
end
end