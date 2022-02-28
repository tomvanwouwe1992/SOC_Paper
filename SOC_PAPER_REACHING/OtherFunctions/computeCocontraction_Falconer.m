function [co_contraction] = computeCocontraction_Falconer(a1,a2)
act(:,:,1) = a1;
act(:,:,2) = a2;

        co_contraction = 2*min(act,[],3)./(a1+a2);

end