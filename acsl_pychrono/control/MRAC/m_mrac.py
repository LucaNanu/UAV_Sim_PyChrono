import math
import numpy as np  

class M_MRAC:
  @staticmethod
  def computeAdaptiveLaw(Gamma_gain, pi_vector, eTranspose_P_B):
    K_hat_state_dot = Gamma_gain * pi_vector * eTranspose_P_B

    return K_hat_state_dot
  
  @ staticmethod
  def compute_eTransposePB(e, P, B):
    eTranspose_P_B = e.T * P * B
    return eTranspose_P_B

  @staticmethod
  def computeAllAdaptiveLaws(
    Gamma_x,
    x,
    Gamma_r,
    r,
    Gamma_Theta,
    Phi_regressor_vector,
    eTranspose_P_B
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Compute all the classical MRAC adaptive laws

    Returns:
      - (K_hat_x_dot, K_hat_r_dot, Theta_hat_dot)
    """

    K_hat_x_dot = M_MRAC.computeAdaptiveLaw(-Gamma_x, x, eTranspose_P_B)
    K_hat_r_dot = M_MRAC.computeAdaptiveLaw(-Gamma_r, r, eTranspose_P_B)
    Theta_hat_dot = M_MRAC.computeAdaptiveLaw(Gamma_Theta, Phi_regressor_vector, eTranspose_P_B)

    return (K_hat_x_dot, K_hat_r_dot, Theta_hat_dot)