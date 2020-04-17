import numpy as np

######## 시스템 정의 ########
 ## x = f(x,u) + w = Fx + Bu + w
 ## z = h(x) + v = Hx + v
def F(x):
    return np.eye(3)

def B(x):
    c=np.cos(x[2])
    s=np.sin(x[2])
    return [[c,-s,0],[s,c,0],[0,0,1]]
    
def mod2pi(theta):
    theta=np.remainder(theta,2*np.pi)
    if theta>np.pi:
        theta-=2*np.pi
    return theta
    
def f(x,u):
    fx=F(x)@x+B(x)@u
#     fx[2]=mod2pi(fx[2])
    return fx

def H(x):
    return np.eye(3)

def h(x):
    return H(x)@x


######## 보조 함수 ########
def AXAT(A,X):
    return A@X@np.transpose(A)

def cov_sample(k):
    return k.reshape(-1,1)@k.reshape(1,-1)

 ## find X for AXA^T=B
def solve_AXAT(A,B):
    vecX,_,_,_=np.linalg.lstsq(np.kron(A,A),np.reshape(B,-1),rcond=None)
    return np.reshape(vecX,(A.shape[1],A.shape[1]))

 ## 정사각행렬을 가까운 Symmetric Positive-Definite Matrix 로 바꿔줌
 ## 공분산 행렬은 SPD여야 하기 때문에 numerical한 문제를 방지하기 위함
def toSPD(A):
    small=0.0001
    A=(A+np.transpose(A))/2
    v,Q=np.linalg.eigh(A)
    v[v<small]=small
    return AXAT(Q,np.diag(v))
def AXAT(A,X):
    return A@X@np.transpose(A)

def cov_sample(k):
    return k.reshape(-1,1)@k.reshape(1,-1)

 ## find X for AXA^T=B
def solve_AXAT(A,B):
    vecX,_,_,_=np.linalg.lstsq(np.kron(A,A),np.reshape(B,-1),rcond=None)
    return np.reshape(vecX,(A.shape[1],A.shape[1]))

 ## 정사각행렬을 가까운 Symmetric Positive-Definite Matrix 로 바꿔줌
 ## 공분산 행렬은 SPD여야 하기 때문에 numerical한 문제를 방지하기 위함
def toSPD(A):
    small=0.0001
    A=(A+np.transpose(A))/2
    v,Q=np.linalg.eigh(A)
    v[v<small]=small
    return AXAT(Q,np.diag(v))


######## 칼만 필터 ########
def predict(x, P, u, Q):
    P=AXAT(F(x),P)+Q
    x=f(x,u)
    return x, P

def correct(x, P, z, R):
    y=z-h(x)
    S=AXAT(H(x),P)+R
    K=P@np.transpose(H(x))@np.linalg.inv(S)
    P=(np.eye(P.shape[0])-K@H(x))@P
    x=x+K@y
    return x, P


######## 가상 데이터 제너레이터(x_real이 0인 경우에만 제대로 작동) ########
def control(x_real, Q_real):
    Binv=np.linalg.inv(B(x_real))
    return np.random.multivariate_normal(np.zeros(np.shape(Q)[0]),AXAT(Binv,Q_real))

def observe(x_real, R_real):
    return np.random.multivariate_normal(h(x_real),R_real)


######## 공분산 추정하면서 적용하는 예시 ########
def predict_z(z,x,u):
    z+=H(x)@B(x)@u
    return z

Q_real=np.eye(3)*0.5
R_real=np.eye(3)*1

x_real=np.zeros(3)
x=np.zeros(3)
P=np.eye(3)
Q=np.eye(3)*1
R=np.eye(3)*0.5

xs=[]
z_olds=[observe(x_real,R_real),observe(x_real,R_real)]
for i in range(10000):
    for j in range(10):
        u=control(x_real,Q_real)
        x,P=predict(x,P,u,Q)
        z_olds[0] = predict_z(z_olds[0],x,u)
        z_olds[1] = predict_z(z_olds[1],x,u)
    z=observe(x_real,R_real)
    
    ## 노이즈 공분산 Q, R 추정
    zAC0 = cov_sample(z-z_olds[0])
    zAC1 = cov_sample(z-z_olds[1])
    
    alpha = 1./min(i+10,1000)
    Qsample = solve_AXAT( H(x)@B(x), (zAC1-zAC0)/(j+1) )
    Q *= (1-alpha)
    Q += alpha * Qsample
    Q=toSPD(Q)
    Rsample = zAC0 - 0.5*zAC1
    R *= (1-alpha)
    R += alpha * Rsample
    R=toSPD(R)
    
    z_olds[1]=z_olds[0]
    z_olds[0]=z
    ##
    
    x,P=correct(x,P,z,R)
    xs.append(x)
    
print("각 성분의 오차 : ")
print((np.transpose(xs)@xs)/(len(xs)-1))

print("추정한 오차 : ")
print(P)

print("노이즈 공분산 실제 vs 추정")
print(Q_real)
print(Q)
print(R_real)
print(R)