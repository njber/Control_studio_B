function [W,F,Phi,Lambda] = MPC_matrices(A,B,Q,R,P,N)
    n=length(A);
    m=size(B)*[0 1]'
    QN=[];
    RN=[];
    Lambda=[];
    Nm = N*m
    Nn = N*n
    for k=1:N
        Lambda=[Lambda;A^k];
        RN=blkdiag(RN,R);
        if (k<N)
            QN=blkdiag(QN,Q);
        else
            QN=blkdiag(QN,P);
        end
    end
    
    Phi=zeros(N*n,N*m);
    row=(N-1)*n+1;
    for k=N:-1:1
        col=1;
        for j=1:k
            Phi(row:row+n-1,col:col+m-1)=(A^(k-j))*B;
            col=col+m;
        end
        row=row-n;
    end

    

    
    W=Phi'*QN*Phi + RN;
    W=(W+W')/2;
    F=Phi'*QN*Lambda;
end