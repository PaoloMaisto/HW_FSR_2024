function S = skew(v)
if(numel(v)~= 1)
S= [0 -v(3) v(2); 
    v(3) 0 -v(1);
    -v(2) v(1) 0];
else
S= zeros(3);
end
end
