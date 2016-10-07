function [ u ] = getControlTraj( pointerMatrix )
    u = pointerMatrix(1,1);
    for i=2:size(pointerMatrix,2)
        u = [u pointerMatrix(u(end),i)];
    end
end

