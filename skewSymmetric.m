function skew = skewSymmetric(a,b,c)

skew = zeros(3,3);
skew(1,2) = -c;
skew(1,3) = b;
skew(2,1) = c;
skew(2,3) = -a;
skew(3,1) = -b;
skew(3,2) = a;

end