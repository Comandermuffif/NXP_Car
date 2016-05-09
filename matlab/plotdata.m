function plotdata(trace, cam)
drawnow;
subplot(4,2,cam);
%figure(figureHandle);
plot(trace);
%set(figureHandle,'Visible','on');

%SMOOTH AND PLOT
smoothtrace = trace;
for i = 3:126
    %5-point Averager
    smoothtrace(i) = mean(trace(i-2:i+2));
end;
subplot(4,2,cam+2);
%figure(smoothhand);
plot(smoothtrace);

%THRESHOLD
%calculate 1's and 0's via thresholding
maxval = max(smoothtrace);
bintrace = zeros(1, length(trace));
avg = mean(smoothtrace);
for i = 1:128
    %Edge detection (binary 0 or 1)
    bintrace(i) = 0;
    if(smoothtrace(i) > avg)
        bintrace(i) = 1;
    end
end
drawnow;
subplot(4,2,cam+4);
%figure(binfighand);
plot(bintrace);

end %function