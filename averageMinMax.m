function [average] = averageMinMax(array, min, max)

n = length(array);
sum =0;

for i = 1 : n
    if array(i) >= min && array(i) <= max
        sum = sum + array(i);
    else
        n = n - 1;
    end
end

average = sum / n;
end

