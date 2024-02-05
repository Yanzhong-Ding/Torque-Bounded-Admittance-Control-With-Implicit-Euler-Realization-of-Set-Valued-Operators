function result=project(Z,x)
a=min(Z);
b=max(Z);
if x>=b
    result=b;
elseif x<=a
    result=a;
else 
    result=x;
end
end