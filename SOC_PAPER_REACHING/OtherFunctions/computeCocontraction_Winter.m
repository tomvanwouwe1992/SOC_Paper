function [co_contraction] = computeCocontraction_Winter(a1,a2)
act(:,:,1) = a1;
act(:,:,2) = a2;

        co_contraction = min(act,[],3)./max(act,[],3).*(a1+a2);

end