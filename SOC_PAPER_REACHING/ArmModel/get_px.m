function px = get_px(q2,auxdata,q1)
p_full = EndEffectorPos([q1;q2],auxdata);
px = p_full(1);
end