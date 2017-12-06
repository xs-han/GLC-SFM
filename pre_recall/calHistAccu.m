data = importdata('surf64_300_6_4_dis.txt');

bins = 0.01:0.02:0.51;
N = hist(data(:,3), 0.01:0.02:0.51);
cumN = cumsum(N) / length(data);

bar(bins, N);
hold on;
plotyy(bins, N , bins, cumN);
hold off;