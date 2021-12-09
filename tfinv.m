function To = tfinv(Ti)

To = [Ti(1:3,1:3)', -Ti(1:3,1:3)'*Ti(1:3, 4); 0 0 0 1];

end

