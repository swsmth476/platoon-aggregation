function input2 = u2(t,h,p)
% input for lead vehicle 2
% h is the headway in front of this vehicle

if(h > 50)
    input2 = v2(t); % - g_fn(h, p.dh, p.k); % cancel out spring when headway is large
else
    input2 = v2(t); % if headway is small, do not cancel out spring
end

end