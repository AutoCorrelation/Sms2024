function output = lowpassFilter(save,input,alpha)
    output = alpha*save + (1-alpha)*input;
end