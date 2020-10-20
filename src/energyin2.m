timed = load('Messures/robotJointTime.txt');
ts = timed(3)-timed(2);

for di = 1:6

    L = length(timed);
    Fs = 1/ts;
    %%%%%
    
    fd_original = yD(di,:);
    fd_error = yD(di,:) - ys(di,:) ;

    P2 = fft(fd_original);
    Fd_original =  P2(1:L/2+1);
    f_fourier = Fs*(0:(L/2))/L;

    % Find the total energy in frequency
    Ed_or = sum(abs(Fd_original(f_fourier<40)).^2);
    % Find the total energy in time
    Edtime_or = sum(abs(fd_original).^2);
    
    P2 = fft(fd_error);
    Fd_error =  P2(1:L/2+1);
    f_fourier = Fs*(0:(L/2))/L;

    % Find the total energy in frequency
    Ed_error = sum(abs(Fd_error(f_fourier<40)).^2);
    % Find the total energy in time
    Edtime_error = sum(abs(fd_error).^2);
    
    
    % Display Energy
    % disp(['Ed = ' num2str(Ed)]);
    % disp(['Ed_time = ' num2str(Edtime)]);


    %%
    % disp(['Acc = ' num2str(Ed/Ed_or)]);
    % disp(['AccTim = ' num2str(Edtime/Edtime_or)]);

    disp(['Error = ' num2str((Ed_error)/Ed_or)]);
    disp(['ErrorTim = ' num2str((Edtime_error)/Edtime_or)]);
end