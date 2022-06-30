

ass guidance_moon():
  def __init__(self, x0, xdes, n, T):  #x: 상태변수
    self.A = np.eye(6)
    self.B = np.zeros((6,3))
    self.T = T
    self.n = n
    self.dt = self.T/self.n
    for i in range(3):
      self.A[i,i+3]=self.dt
      self.B[i,i]=self.dt**2/2
      self.B[i+3,i]= self.dt
    self.x_0 = x0
    self.x_des = xdes
    self.x = np.zeros((6,n+1))
    self.x[:,0]=self.x_0
    self.G = np.zeros((6,3*self.n))
    self.P = np.zeros((6,6*self.n))
    self.g = 1.63 #달의 중력가속도
    self.make_matrix()
    self.opt_tr()
    self.lamda2 = 1000

  def make_matrix(self):
    temp = -0.5*self.g*self.dt**2
    self.b = [0, 0, temp, 0, 0, -self.g*self.dt]
    self.bs = self.b* self.n
    for i in range(self.n):
      self.G[:,3*i:3*(i+1)] = np.linalg.matrix_power(self.A,self.n-(i+1))@self.B
    for i in range(self.n):
      self.P[:,6*i:6*(i+1)]=np.linalg.matrix_power(self.A, n-(i+1))
    self.Q = self.P@self.bs

  def opt_tr(self):
    self.first = np.linalg.matrix_power(self.A, self.n)@self.x_0
    self.u_tilde = sla.lsqr(self.G, self.x_des - self.first-self.Q )[0]
    print(self.u_tilde.shape)
    self.u = self.u_tilde.reshape(self.n,3) #0~n-1

    for i in range(self.n):
      self.x[:,i+1]=self.A.dot(self.x[:,i])+self.B.dot(self.u[i,:])+self.b

  def lamda_matrix(self, lamda):
    self.G_tilde = np.concatenate((self.G[0:3,:],self.lamda2*self.G[3:6,:]),axis=0)
    print(self.G_tilde.shape)
    self.xqf = self.x_des-self.Q-self.first
    self.xqf_tilde = np.zeros(6)
    self.xqf_tilde[0:3] = self.xqf[0:3]
    self.xqf_tilde[3:6] = self.lamda2*self.xqf[3:6]
    self.A_tilde = np.concatenate((self.G_tilde,np.sqrt(lamda)*np.eye(3*self.n)),axis=0)
    self.b_tilde = np.concatenate((self.xqf_tilde,np.zeros(3*self.n)), axis=0)

  def make_svec(self, us):
    self.x_gd = np.empty((6,0))
    self.x_gd = np.append(self.x_gd, self.x_0[:,np.newaxis],axis=1)
    self.us = us.reshape(self.n, 3)
    self.us = np.transpose(self.us)

    for i in range(self.n):
      temp_x = self.A@self.x_gd[:,i] +self.B@self.us[:,i]+self.b
      self.x_gd = np.append(self.x_gd, temp_x[:,np.newaxis],axis=1)

T = 600
n = 600
dt = T/n
x_des = np.array([0,0,0,0,0,0])
x_0 =np.array([10000,10000,15000,-1500/math.sqrt(2),-1500/math.sqrt(2),0])

ts = np.linspace(0,T,n+1)
u_lb = 0
u_ub = 8

guidance = guidance_moon(x_0, x_des, n, T)
guidance.lamda_matrix(100)

optimizer1 = constrained_optimizer(guidance.A_tilde,guidance.b_tilde[:np.newaxis], guidance.u_tilde,0,8,1000,0.01,0.0001,n=n,k2_max=100)   #input: 1차원벡터
start = np.random.randn(1800)
#optimizer1 = constrained_optimizer(guidance.A_tilde,guidance.b_tilde[:np.newaxis], start,0,10,1000,0.01,0.000001,n=n,k2_max=100)   #input: 1차원벡터
optimizer1.gradients()

guidance.make_svec(optimizer1.point)


