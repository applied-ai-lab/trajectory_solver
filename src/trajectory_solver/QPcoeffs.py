class QPcoeffs:
    def __init__(self, P=None, q=None, A=None, l=None, u=None):
        self._P = P
        self._q = q
        self._A = A
        self._l = l
        self._u = u

    # Getters 
    @property
    def P(self):
        return self._P

    @property
    def q(self):
        return self._q
    
    @property
    def A(self):
        return self._A
    
    @property
    def l(self):
        return self._l
    
    @property
    def u(self):
        return self._u
    
    # Setters
    @P.setter
    def P(self, P_mat):
        self._P = P_mat
    
    @q.setter
    def q(self, q_vec):
        self._q = q_vec

    @A.setter
    def A(self, A_mat):
        self._A = A_mat

    @l.setter
    def l(self, l_vec):
        self._l = l_vec

    @u.setter
    def u(self, u_vec):
        self._u = u_vec
