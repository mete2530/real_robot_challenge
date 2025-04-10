import numpy as np

class KalmanFilter:
    def __init__(self):
        ## State Matrix
        self.x = np.array([[0], 
                           [0], 
                           [0]])

        ## Covariance Matrix
        self.sigma = np.eye(3) * 0.1

        ## Process Noise Covariance Matrix
        self.R = np.eye(3) * 0.01

        ## Measurement Noise Covariance Matrix
        self.Q = np.eye(3) * 0.1 

        ## State Transition Matrix
        self.A = np.eye(3)

        ## Measurement Model Matrix
        self.C = np.eye(3)

    @staticmethod
    def normalize_angle(angle : float):
        '''Helper function to Normalize the angle between -pi to pi.'''
        return np.arctan2(np.sin(angle), np.cos(angle))

    def predict(self, v : float, w : float, dt : float):
        """Predict the state and covariance matrix based on control inputs v and w.
        v: linear velocity
        w: angular velocity
        dt: time step
        """
        ## TODO: Implement the Prediction Step and remove the NotImplementedError after implementing.
        ## NOTE: Don't forget to normalize the angle after each modification to state.

        ## Implement the control model.

        ## Predict the state.

        ## Predict the Covariance Matrix

        raise NotImplementedError("Prediction step not implemented yet.")

    
    def update(self, z : np.ndarray):
        '''Update the state and covariance matrix based on z.
        Args:
            z: measurement vector of shape (3, 1)
        '''
        ## TODO: Implement the Update Step and remove the NotImplementedError after implementing.
        ## NOTE: Don't forget to normalize the angle after each modification to state.

        ## Compute the Kalman Gain.

        ## Compute Innovation

        ## Update the state.

        ## Update the Covariance Matrix

        raise NotImplementedError("Update step not implemented yet.")

if __name__ == '__main__':
    kf = KalmanFilter()

    ## Test your implementation here.
    ## This is just a sanity check to see if the functions are without errors.
    
    ## Test Prediction
    ## Exptected output: State after prediction: [[0.0], [0.0], [0.0]]
    kf.predict(0.0, 0.0, 0.1)
    print("State after prediction:", kf.x)

    ## Test Update
    ## Expected output: State after update: [[0.0], [0.0], [0.0]]
    kf.update(np.array([[0.0], 
                        [0.0], 
                        [0.0]]))
    print("State after update:", kf.x)