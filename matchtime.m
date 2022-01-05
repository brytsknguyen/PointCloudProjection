function [idxA, idxB] = matchtime(ta, tb, tbound, tmax)

idxA = [];
idxB = [];

last_match_idx = 1;
for n=1:size(ta, 1)
    for m=last_match_idx:size(tb, 1)

        if abs(ta(n) - tb(m)) < tbound
            idxA = [idxA; n];
            idxB = [idxB; m];
            last_match_idx = m + 1;
            break;
        elseif (tb(m) - ta(n)) > tmax
            break;
        end

    end
end

end

