function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise


    in1 = inpolygon(P2(1,1), P2(1,2), P1(:, 1), P1(:, 2));
    in2 = inpolygon(P2(2,1), P2(2,2), P1(:, 1), P1(:, 2));
    in3 = inpolygon(P2(3,1), P2(3,2), P1(:, 1), P1(:, 2));
    
    in4 = inpolygon(P1(1,1), P1(1,2), P2(:, 1), P2(:, 2));
    in5 = inpolygon(P1(2,1), P1(2,2), P2(:, 1), P2(:, 2));
    in6 = inpolygon(P1(3,1), P1(3,2), P2(:, 1), P2(:, 2));    
    
    saidas=[in1, in2, in3, in4, in5, in6];
    flag = any(saidas(:)==1);
end