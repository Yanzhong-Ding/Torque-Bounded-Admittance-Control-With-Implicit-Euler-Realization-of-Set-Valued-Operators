function result=SignFn(parameter1)
if (parameter1<-1e-4)
   result = -1;
elseif (parameter1>1e-4)
    result = 1;
else
   result = parameter1/1e-4;
end
end