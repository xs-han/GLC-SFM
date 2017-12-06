grdth = importdata('KeyFrameTrajectory.txt');
thredis = 1.6;
thresmch = 30;

input_a = importdata('surf64_lab_Geometric.txt');
r1_a = 0;
for la = input_a'
    r1_a = r1_a + 1;
    if(la(5,1) > 80)
        break;
    end
end
r3_a = 0;
for la = input_a'
    r3_a = r3_a + 1;
    if(la(5,1) > 395)
        break;
    end
end
[npoints_a,~] = size(input_a);
res_a = [];
for threshold = 1:-0.01:0
    ndetect = 0;
    ndetect_right = 0;

    for i = 1:r3_a
        m1 = input_a(i,1); t1 = input_a(i,5); loc1 = findGT(t1, grdth);
        m2 = input_a(i,2); t2 = input_a(i,6); loc2 = findGT(t2, grdth);
        ref = input_a(i,3);
        score = input_a(i, 4);
        nmch = input_a(i,7);
        
        if(score / ref > threshold)
            ndetect = ndetect + 1;
            if(isempty(rightarray))
                continue;
            end
            if(norm(loc1 - loc2) < thredis && nmch > thresmch)
                ndetect_right = ndetect_right + 1;
            end
        end
    end

    if(ndetect == 0)
        precision = 1;
        recall = 0;
    else
        precision = ndetect_right / ndetect;
        recall = ndetect_right / (r3_a - r1_a);
    end
    res_a = [res_a;[threshold, ndetect, precision, recall]];
end

input_b = importdata('surf_lab_Geometric.txt');
r1_b = 0;
for lb = input_b'
    r1_b = r1_b + 1;
    if(lb(5,1) > 80)
        break;
    end
end
r3_b = 0;
for lb = input_b'
    r3_b = r3_b + 1;
    if(lb(5,1) > 395)
        break;
    end
end
[npoints_b,~] = size(input_b);
res_b = [];
for threshold = 1:-0.01:0
    ndetect = 0;
    ndetect_right = 0;

    for i = 1:r3_b
        m1 = input_b(i,1); t1 = input_b(i,5); loc1 = findGT(t1, grdth);
        m2 = input_b(i,2); t2 = input_b(i,6); loc2 = findGT(t2, grdth);
        ref = input_b(i,3);
        score = input_b(i, 4);
        nmch = input_b(i,7);
        
        if(score / ref > threshold)
            ndetect = ndetect + 1;
            if(isempty(rightarray))
                continue;
            end
            if(norm(loc1 - loc2) < thredis && nmch > thresmch)
                ndetect_right = ndetect_right + 1;
            end
        end
    end

    if(ndetect == 0)
        precision = 1;
        recall = 0;
    else
        precision = ndetect_right / ndetect;
        recall = ndetect_right / (r3_b - r1_b);
    end

    res_b = [res_b;[threshold, ndetect, precision, recall]];
end

input_c = importdata('sift_lab_Geometric.txt');
r1_c = 0;
for lb = input_c'
    r1_c = r1_c + 1;
    if(lb(5,1) > 80)
        break;
    end
end
r3_c = 0;
for lb = input_c'
    r3_c = r3_c + 1;
    if(lb(5,1) > 395)
        break;
    end
end
[npoints_c,~] = size(input_c);
res_c = [];
for threshold = 1:-0.01:0
    ndetect = 0;
    ndetect_right = 0;

    for i = 1:r3_c
        m1 = input_c(i,1); t1 = input_c(i,5); loc1 = findGT(t1, grdth);
        m2 = input_c(i,2); t2 = input_c(i,6); loc2 = findGT(t2, grdth);
        ref = input_c(i,3);
        score = input_c(i, 4);
        nmch = input_c(i,7);
        
        if(score / ref > threshold)
            ndetect = ndetect + 1;
            if(isempty(rightarray))
                continue;
            end
            if(norm(loc1 - loc2) < thredis && nmch > thresmch)
                ndetect_right = ndetect_right + 1;
            end
        end
    end

    if(ndetect == 0)
        precision = 1;
        recall = 0;
    else
        precision = ndetect_right / ndetect;
        recall = ndetect_right / (r3_c - r1_c);
    end

    res_c = [res_c;[threshold, ndetect, precision, recall]];
end

input_d = importdata('orb_lab_Geometric.txt');
r1_d = 0;
for lb = input_d'
    r1_d = r1_d + 1;
    if(lb(5,1) > 80)
        break;
    end
end
r3_d = 0;
for lb = input_d'
    r3_d = r3_d + 1;
    if(lb(5,1) > 395)
        break;
    end
end
[npoints_d,~] = size(input_d);
res_d = [];
for threshold = 1:-0.01:0
    ndetect = 0;
    ndetect_right = 0;

    for i = 1:r3_d
        m1 = input_d(i,1); t1 = input_d(i,5); loc1 = findGT(t1, grdth);
        m2 = input_d(i,2); t2 = input_d(i,6); loc2 = findGT(t2, grdth);
        ref = input_d(i,3);
        score = input_d(i, 4);
        nmch = input_d(i,7);
        
        if(score / ref > threshold)
            ndetect = ndetect + 1;
            if(isempty(rightarray))
                continue;
            end
            if(norm(loc1 - loc2) < thredis && nmch > thresmch)
                ndetect_right = ndetect_right + 1;
            end
        end
    end

    if(ndetect == 0)
        precision = 1;
        recall = 0;
    else
        precision = ndetect_right / ndetect;
        recall = ndetect_right / (r3_d - r1_d);
    end

    res_d = [res_d;[threshold, ndetect, precision, recall]];
end

xlabel('precision');
ylabel('recall rate');
hold on;
plot(res_a(:,3),res_a(:,4),'-*');
plot(res_b(:,3),res_b(:,4),'-*');
plot(res_c(:,3),res_c(:,4),'-*');
plot(res_d(:,3),res_d(:,4),'-*');
legend('surf64','surf128','sift','orb');
hold off;
