fileID = fopen('testfile.txt','r');

formatSpec = '%d %d %d';
A = fscanf(fileID,formatSpec);
A1 = reshape(A, 3, [])';

fclose(fileID);


figure("Color", "white");
hold on;
for k = 1:1e3
    quality = A1(k, 1)/64;
    angle = A1(k, 2);
    distance = A1(k,3);

    x = cos(deg2rad(angle)) * distance;
    y = sin(deg2rad(angle)) * distance;

    col_good = [0, 1, 0];
    col_bad = [1,0,0];

    col = col_bad * (1-quality) + col_good * quality;

    plot(x, y, "o", "Color", col);
end