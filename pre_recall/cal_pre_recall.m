input_a = importdata('surf_lab.txt');
input_b = importdata('surf_lab_virtual.txt');
r1 = 153;
r2 = 355;
r3 = 503;

thresmch = 20;
thresdis = 30;

ans_ab = cell(r3,1);
for i = 1:length(ans_ab)
    if(i <= r1 - 5)
        ans_ab{i} = [];
    elseif(r1 - 5 <= i && i <= r1)
        ans_ab{i} = [0];
    elseif((i > r1) && (i <= r2))
        ans_ab{i} = [round((i - r1) / (r2 - r1) * r1) - 1];
    elseif((i > r2) && (i <= r3))
        ans_ab{i} = [round((i - r2) / (r3 - r2) * r1) - 1, round((i - r2) / (r3 - r2) * (r2 - r1) + r1) - 1];
    end
end

[npoints_a,~] = size(input_a);
res_a = [];
for threshold = 1:-0.01:0
    ndetect = 0;
    ndetect_right = 0;

    for i = 1:npoints_a
        m1 = input_a(i,1);
        m2 = input_a(i,2);
        ref = input_a(i,3);
        score = input_a(i, 4);
        nmch = input_a(i,5);
        
        if(score / ref > threshold)
            ndetect = ndetect + 1;
            rightarray = ans_ab{m1+1};
            if(isempty(rightarray))
                continue;
            end
            
            for an_right_ans_ab = rightarray
                if(abs(m2 - an_right_ans_ab) < thresdis  && nmch > thresmch)
                    ndetect_right = ndetect_right + 1;
                    break;
                end
            end
        end
    end

    if(ndetect == 0)
        precision = 1;
        recall = 0;
    else
        precision = ndetect_right / ndetect;
        recall = ndetect_right / (r3 - r1);
    end

    res_a = [res_a;[threshold, ndetect, precision, recall]];
end

[npoints_b,~] = size(input_b);
res_b = [];
for threshold = 1:-0.01:0
    ndetect = 0;
    ndetect_right = 0;

    for i = 1:npoints_b
        m1 = input_b(i,1);
        m2 = input_b(i,2);
        ref = input_b(i,3);
        score = input_b(i, 4);
        nmch = input_b(i,5);
        
        if(score / ref > threshold)
            ndetect = ndetect + 1;
            rightarray = ans_ab{m1+1};
            if(isempty(rightarray))
                continue;
            end
           
            
            for an_right_ans_ab = rightarray
                if(abs(m2 - an_right_ans_ab) < thresdis  && nmch > thresmch)
                    ndetect_right = ndetect_right + 1;
                    break;
                end
            end
        end
    end

    if(ndetect == 0)
        precision = 1;
        recall = 0;
    else
        precision = ndetect_right / ndetect;
        recall = ndetect_right / (r3 - r1);
    end

    res_b = [res_b;[threshold, ndetect, precision, recall]];
end

plot(res_a(:,3),res_a(:,4),'-o');
xlabel('precision');
ylabel('recall rate');
hold on;
plot(res_b(:,3),res_b(:,4),'-*');
legend('DBoW','Ours');
hold off;
