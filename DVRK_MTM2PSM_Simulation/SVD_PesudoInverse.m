function [A_pesudo]=SVD_PesudoInverse(A)
[U,S,V] = svd(A);
[m,n]=size(S);
S_pesudo=S;
for i=1:min(m,n)
    if S(i,i)>0 || S(i,i)<0
        S_pesudo(i,i)=1/S(i,i);
    end
end
A_pesudo=V*S_pesudo'*U';
end