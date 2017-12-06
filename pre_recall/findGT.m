function loc = findGT(t, grdth)
tint = round(t);
if(tint > length(grdth))
    tint = length(grdth);
    loc = grdth(tint, 2:4);
    return;
end
if(grdth(tint, 1) > t)
    for i = 1:tint-1
        if(grdth(tint - i, 1) < t)
            loc = (grdth(tint - i, 2:4) * abs(grdth(tint - i, 1) - t) + grdth(tint - i + 1, 2:4) * abs(grdth(tint - i + 1, 1) - t) ) / (abs(grdth(tint - i + 1, 1) - grdth(tint - i, 1)));
            break;
        end
    end
else
    for i = 1:length(grdth) - tint
        if(grdth(tint + i, 1) > t)
            loc = (grdth(tint + i, 2:4) * abs(grdth(tint + i, 1) - t) + grdth(tint + i - 1, 2:4) * abs(grdth(tint + i - 1, 1) - t)) / (abs(grdth(tint + i - 1, 1) - grdth(tint + i, 1)));
            break;
        end
    end
end

end
