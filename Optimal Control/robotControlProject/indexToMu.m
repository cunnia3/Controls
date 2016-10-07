function [ mu ] = indexToMu( index,maxIndex,maxMu )
    mu = (index-1)/maxIndex*maxMu;
end

