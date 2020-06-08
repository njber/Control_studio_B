function stop=check_matrices(W,F,Lambda,AN,Umax,Umin,Xmax,Xmin,N,n,m)
    stop=0;
    header = 0;
        
    if ((size(W)*[1 0]'~=N*m) || (size(W)*[0 1]'~=N*m))
               disp(' ')
               disp('This is a friendly message from Ricardo!!!')
               disp(' ')
               disp(['Error in dimension of Matrix W: ',int2str(size(W)*[1 0]'),'x',int2str(size(W)*[0 1]')])
               disp(['For N=',int2str(N),', n=',int2str(n),', m=',int2str(m),', W must be ', int2str(N*m),'x',int2str(N*m) ])
               stop=1;
               header=1;
    end
    
    if ((size(F)*[1 0]'~=N*m) || (size(F)*[0 1]'~=n))
        if (header==0)
            disp(' ')
            disp('This is a friendly message from Ricardo!!!')
        end
        disp(' ')
        disp(['Error in dimension of Matrix F: ',int2str(size(F)*[1 0]'),'x',int2str(size(F)*[0 1]')])
        disp(['For N=',int2str(N),', n=',int2str(n),', m=',int2str(m),', F must be ', int2str(N*m),'x',int2str(n) ])
        stop=1;
        header=1;
    end
    if ((size(Lambda)*[1 0]'~=N*n) || (size(Lambda)*[0 1]'~=n))
        if (header==0)
            disp(' ')
            disp('This is a friendly message from Ricardo!!!')
        end
        disp(' ')
        disp(['Error in dimension of Matrix Lambda: ',int2str(size(Lambda)*[1 0]'),'x',int2str(size(Lambda)*[0 1]')])
        disp(['For N=',int2str(N),', n=',int2str(n),', m=',int2str(m),', Lambda must be ', int2str(N*n),'x',int2str(n) ])
        stop=1;
        header=1;
    end
    if ((size(AN)*[1 0]'~=2*N*(n+m)) || (size(AN)*[0 1]'~=N*m))
        if (header==0)
            disp(' ')
            disp('This is a friendly message from Ricardo!!!')
        end
        disp(' ')
        disp(['Error in dimension of Matrix AN: ',int2str(size(AN)*[1 0]'),'x',int2str(size(AN)*[0 1]')])
        disp(['For N=',int2str(N),', n=',int2str(n),', m=',int2str(m),', AN must be ', int2str(2*N*(n+m)),'x',int2str(N*m) ])
        stop=1;
        header=1;
    end
    if ((size(Umax)*[1 0]'~=N*m) || (size(Umax)*[0 1]'~=1))
        if (header==0)
            disp(' ')
            disp('This is a friendly message from Ricardo!!!')
        end
        disp(' ')
        disp(['Error in dimension of Matrix Umax: ',int2str(size(Umax)*[1 0]'),'x',int2str(size(Umax)*[0 1]')])
        disp(['For N=',int2str(N),', n=',int2str(n),', m=',int2str(m),', Umax must be ', int2str(N*m),'x1'])
        stop=1;
        header=1;
    end
    if ((size(Umin)*[1 0]'~=N*m) || (size(Umin)*[0 1]'~=1))
        if (header==0)
            disp(' ')
            disp('This is a friendly message from Ricardo!!!')
        end
        disp(' ')
        disp(['Error in dimension of Matrix Umin: ',int2str(size(Umin)*[1 0]'),'x',int2str(size(Umin)*[0 1]')])
        disp(['For N=',int2str(N),', n=',int2str(n),', m=',int2str(m),', Umin must be ', int2str(N*m),'x1'])
        stop=1;
        header=1;
    end
    if ((size(Xmax)*[1 0]'~=N*n) || (size(Xmax)*[0 1]'~=1))
        if (header==0)
            disp(' ')
            disp('This is a friendly message from Ricardo!!!')
        end
        disp(' ')
        disp(['Error in dimension of Matrix Xmax: ',int2str(size(Xmax)*[1 0]'),'x',int2str(size(Xmax)*[0 1]')])
        disp(['For N=',int2str(N),', n=',int2str(n),', m=',int2str(m),', Xmax must be ', int2str(N*n),'x1'])
        stop=1;
        header=1;
    end
    if ((size(Xmin)*[1 0]'~=N*n) || (size(Xmin)*[0 1]'~=1))
        if (header==0)
            disp(' ')
            disp('This is a friendly message from Ricardo!!!')
        end
        disp(' ')
        disp(['Error in dimension of Matrix Xmin: ',int2str(size(Xmin)*[1 0]'),'x',int2str(size(Xmin)*[0 1]')])
        disp(['For N=',int2str(N),', n=',int2str(n),', m=',int2str(m),', Xmin must be ', int2str(N*n),'x1'])
        stop=1;
        header=1;
    end
    
    
    
    if (stop==1)
        disp(' ')
        disp('Simulation has been stopped')
        disp('Keep up the hard work!!!')
        disp(' ')
    end
end