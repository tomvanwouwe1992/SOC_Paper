function [co_contraction] = computeCocontraction(a1,a2)
    co_contraction = NaN(length(a1),1);
    for i = 1:length(a1)
        co_contraction(i) = min(a1(i),a2(i))./max(a1(i),a2(i)).*(a1(i)+a2(i));
    end
end