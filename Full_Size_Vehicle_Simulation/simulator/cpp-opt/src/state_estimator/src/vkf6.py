import numpy as np
import scipy.linalg
from copy import deepcopy
from threading import Lock


class UKFException(Exception):
    """Raise for errors in the UKF, usually due to bad inputs"""


class VKF:
    def __init__(self, num_states, num_inputs, initial_state, initial_inputs,initial_covar, alpha, k, beta, iterate_function):
        """
        Initializes the unscented kalman filter
        :param num_states: int, the size of the state
        :param process_noise: the process noise covariance per unit time, should be num_states x num_states
        :param initial_state: initial values for the states, should be num_states x 1
        :param initial_covar: initial covariance matrix, should be num_states x num_states, typically large and diagonal
        :param alpha: UKF tuning parameter, determines spread of sigma points, typically a small positive value
        :param k: UKF tuning parameter, typically 0 or 3 - num_states
        :param beta: UKF tuning parameter, beta = 2 is ideal for gaussian distributions
        :param iterate_function: function that predicts the next state
                    takes in a num_states x 1 state and a float timestep and inputs
                    returns a num_states x 1 state
        :param observation_function: function the translates states to measurements
                    takes in a num_states x 1 state
                    returns a num_observations x 1 measurement
        """
        """
            The following code corresponds to block (15)

            Following the UKF paper by Wan, van der Merwe
            with non additive noise. For simplicity the measurement update step with be a standard EKF update
            self.covar_weights[0]=W_{0}^{c}
            self.mean_weights[0]=W_{0}^{m}
            self.mean_weights[i]=W_{i}^{m}
            self.covar_weights[i]=W_{i}^{c}
            self.p=P_{x}
            self.sigmas=X matrix n_dim x n_sig matrix. column 0 is the inital state
        """
        self.n_dim = int(num_states)
        self.state_dim=int(num_states)
        self.input_dim=int(num_inputs)
        self.n_sig = 1 + self.n_dim  * 2
        self.x = initial_state
        self.p = initial_covar
        self.beta = beta
        self.alpha = alpha
        self.k = k
        self.inputs = initial_inputs
        self.iterate = iterate_function


        #scaling parameter
         
        self.lambd = pow(self.alpha,2) * (self.n_dim + self.k) - self.n_dim

        self.covar_weights = np.zeros(self.n_sig)
        self.mean_weights = np.zeros(self.n_sig)

        self.covar_weights[0] = (self.lambd / (self.n_dim + self.lambd)) + (1 - pow(self.alpha, 2) + self.beta)

        self.mean_weights[0] = (self.lambd / (self.n_dim + self.lambd))

        for i in range(1, self.n_sig):
            self.covar_weights[i] = 1 / (2*(self.n_dim + self.lambd))
            self.mean_weights[i] = 1 / (2*(self.n_dim + self.lambd))

        self.sigmas = self.__get_sigmas()

        self.lock = Lock()
 
    def posalert(self):
        egs = np.linalg.eigvals(self.p)
        neg = False
        for eg in egs:
          if eg<=0 or np.iscomplex(eg):
            neg=True
        if neg:
            c = np.eye(4)
            c[0][0] = 0.0444
            c[0][1] = -0.0029
  	    c[0][2] = 0.0005
            c[0][3] = 0.0422
            c[1][0] = -0.0029
            c[1][1] = 0.0007
            c[1][2] = 0.0038
            c[1][3] = -0.0027
            c[2][0] = 0.0005
            c[2][1] = 0.0038
            c[2][2] = 0.0436
            c[2][3] = 0.0002
            c[3][0] = 0.0422
            c[3][1] = -0.0027
            c[3][2] = -0.0002
            c[3][3] = 0.0434
            result = c
        else:
            result = self.p
        return result

    def __get_sigmas(self):
        """generates sigma points"""
        ret = np.zeros((self.n_sig, self.n_dim))
        tmp_mat = (self.n_dim + self.lambd)*self.posalert()
        spr_mat = scipy.linalg.sqrtm(tmp_mat)
        x_aug = self.x
        ret[0] = x_aug
        for i in range(self.n_dim):
            ret[i+1] = x_aug + spr_mat.T[i]
            ret[i+1+self.n_dim] = x_aug - spr_mat.T[i]
        return ret.T

    def update_inputs(self, data, index=-1):
        """callback function to update inputs
            :param inputs: list of indices of which inputs are being updated
            :param data: data corresponding to input updata
        """
        self.lock.acquire()
        if index >= 0:
	  self.inputs[index] = data
        self.lock.release()
   
    def update(self, states, data, r_matrix):
        """
        performs a measurement update
        :param states: list of indices (zero-indexed) of which states were measured, that is, which are being updated
        :param data: list of the data corresponding to the values in states
        :param r_matrix: error matrix for the data, again corresponding to the values in states
        """

        self.lock.acquire()

        num_states = len(states)

        # create y, sigmas of just the states that are being updated
        sigmas_split = np.split(self.sigmas, self.n_dim)
        y = np.concatenate([sigmas_split[i] for i in states])

        # create y_mean, the mean of just the states that are being updated
        x_split = np.split(self.x, self.n_dim)
        y_mean = np.concatenate([x_split[i] for i in states])

        # differences in y from y mean
        y_diff = deepcopy(y)
        x_diff = deepcopy(self.sigmas)
        for i in range(self.n_sig):
            for j in range(num_states):
                y_diff[j][i] -= y_mean[j]
            for j in range(self.n_dim):
                x_diff[j][i] -= self.x[j]

        # covariance of measurement
        p_yy = np.zeros((num_states, num_states))
        for i, val in enumerate(np.array_split(y_diff, self.n_sig, 1)):
            p_yy += self.covar_weights[i] * val.dot(val.T)

        # add measurement noise
        p_yy += r_matrix

        # covariance of measurement with states
        p_xy = np.zeros((self.n_dim, num_states))
        for i, val in enumerate(zip(np.array_split(y_diff, self.n_sig, 1), np.array_split(x_diff, self.n_sig, 1))):
            p_xy += self.covar_weights[i] * val[1].dot(val[0].T)

        k = np.dot(p_xy, np.linalg.inv(p_yy))

        y_actual = data

        self.x += np.dot(k, (y_actual - y_mean))
        self.p -= np.dot(k, np.dot(p_yy, k.T))
        self.sigmas = self.__get_sigmas()

        self.lock.release()

    def predict(self, timestep):
        """
        performs a prediction step
        :param timestep: float, amount of time since last prediction
        comments with line numbers correspond to line in Algorithm 2.1 in Surat Kwanmuang's PhD thesis

        self.x=x_out=x_{k}^{^-}
        self.sigma=sigmas_out=X_{k}^{-} n_dimxn_sig
        self.p=p_out=P_{k}^{-}
        """

        self.lock.acquire()

        #Line (2.12)
        self.sigmas = self.__get_sigmas()

        inputs=self.inputs
        #line (2.13)
        sigmas_out = np.array([self.iterate(x[range(self.state_dim)], timestep, inputs) for x in self.sigmas.T]).T
        #line (2.14)
        x_out = np.zeros(self.state_dim)
        # for each variable in X

        for i in range(self.state_dim):
            # the mean of that variable is the sum of
            # the weighted values of that variable for each iterated sigma point
            x_out[i] = sum((self.mean_weights[j] * sigmas_out[i][j] for j in range(self.n_sig)))

        #Line (2.15)
        p_out = np.zeros((self.state_dim, self.state_dim))
        # for each sigma point
        for i in range(self.n_sig):
            # take the distance from the mean
            # make it a covariance by multiplying by the transpose
            # weight it using the calculated weighting factor
            # and sum
            diff = sigmas_out.T[i] - x_out
            diff = np.atleast_2d(diff)
            p_out += self.covar_weights[i] * np.dot(diff.T, diff)
        self.x = x_out
        self.p = p_out
        
        self.lock.release()

    def get_state(self, index=-1):
        """
        returns the current state (n_dim x 1), or a particular state variable (float)
        :param index: optional, if provided, the index of the returned variable
        :return:
        """
        if index >= 0:
            return self.x[index]
        else:
            return self.x
    def get_inputs(self,index=-1):
        """
        returns the current input (n_dim x 1), or a particular state variable (float)
        :param index: optional, if provided, the index of the returned variable
        :return:
        """
        if index >= 0:
            return self.inputs[index]
        else:
            return self.inputs

    def get_covar(self):
        """
        :return: current state covariance (n_dim x n_dim)
        """
        return self.p

    def set_state(self, value, index=-1):
        """
        Overrides the filter by setting one variable of the state or the whole state
        :param value: the value to put into the state (1 x 1 or n_dim x 1)
        :param index: the index at which to override the state (-1 for whole state)
        """
        with self.lock:
            if index != -1:
                self.x[index] = value
            else:
                self.x = value
            self.sigmas=self.__get_sigmas()

    def reset(self, state, covar):
        """
        Restarts the UKF at the given state and covariance
        :param state: n_dim x 1
        :param covar: n_dim x n_dim
        """

        self.lock.acquire()
        self.x = state
        self.p = covar
        self.sigmas = self.__get_sigmas()
        self.lock.release()
"""
    def update(self,observations,z_hat,H,R):
        
        performs a measurement update EKF style
        :param observations: list of indices (zero-indexed) of which states were measured, that is, which are being updated
        :param r_matrix: error matrix for the data, again corresponding to the values in states

        Modifed to follow Algorithm 2.1 in  Surat Kwanmuang's PhD thesis
        self.x=x_{k}^{^-}
        self.sigma=X_{k}^{-}  n_dimx n_sig matrix. column 0 is the inital state
        self.p=P_{k}^{-}
        //

        self.lock.acquire()
        z_hat=z_hat(self.x)
        H=H(self.x)
        S=np.dot(np.dot(H,self.p),H.T)+R
        K=np.dot(np.dot(self.p,H.T),scipy.linalg.inv(S))
        #correction step
        self.x=self.x+np.dot(K,observations-z_hat)
        self.p=np.dot(np.eye(self.state_dim)-np.dot(K,H),self.p)
        self.lock.release()
"""

