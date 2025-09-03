# import math
import numpy as np  
from acsl_pychrono.control.outerloop_safetymech import OuterLoopSafetyMechanism
from acsl_pychrono.control.PID.pid_gains import PIDGains
from acsl_pychrono.simulation.ode_input import OdeInput
from acsl_pychrono.simulation.flight_params import FlightParams
from acsl_pychrono.control.control import Control

class PID(Control):
  def __init__(self, gains: PIDGains, ode_input: OdeInput, flight_params: FlightParams, timestep: float):
    super().__init__(odein=ode_input)
    self.gains = gains
    self.fp = flight_params
    self.timestep = timestep
    self.safety_mechanism = OuterLoopSafetyMechanism(gains, self.fp.G_acc)
    self.dy = np.zeros((self.gains.number_of_states, 1))
    # Initial conditions
    self.y = np.zeros((self.gains.number_of_states, 1))

  def computeControlAlgorithm(self, ode_input: OdeInput):
    """
    Compute all intermediate variables and control inputs once per RK4 step to compute the dy for RK4.
    """
    # Update the vehicle state and user-defined trajectory
    self.odein = ode_input
    
    # ODE state
    self.state_phi_ref_diff = self.y[0:2]
    self.state_theta_ref_diff = self.y[2:4]
    self.integral_position_tracking = self.y[4:7]
    self.integral_e_rot = self.y[7:10] # Integral of 'e_rot' = (angular_velocity - omega_ref) 
    self.integral_angular_error = self.y[10:13] # Integral of angular_error = attitude - attitude_ref
    self.integral_e_omega_ref_cmd = self.y[13:16] #Integral of (omega_ref - omega_cmd)
    self.omega_ref = self.y[16:19] # Reference model rotational dynamics
 
    # Compute translational position error
    self.translational_position_error = Control.computeTranslationalPositionError(
      self.odein.translational_position_in_I,
      self.odein.translational_position_in_I_user
    )   
    
    # Compute Outer Loop
    self.computeOuterLoop()

    # Outer Loop Safety Mechanism
    self.mu_x, self.mu_y, self.mu_z = self.safety_mechanism.apply(self.mu_tran_raw)

    # Compute total thrust, desired roll angle, desired pitch angle
    (
    self.u1,
    self.roll_ref,
    self.pitch_ref
    ) = Control.computeU1RollPitchRef(
      self.mu_x, 
      self.mu_y, 
      self.mu_z, 
      self.gains.mass_total_estimated,
      self.fp.G_acc,
      self.odein.yaw_ref
    )

    # Computes roll/pitch reference dot and ddot using state-space differentiators.
    (
    self.internal_state_differentiator_phi_ref_diff,
    self.internal_state_differentiator_theta_ref_diff,
    self.angular_position_ref_dot,
    self.angular_position_ref_ddot
    ) = Control.computeAngularReferenceSignals(
      self.fp,
      self.odein,
      self.roll_ref,
      self.pitch_ref,
      self.state_phi_ref_diff,
      self.state_theta_ref_diff
    )

    # Computes angular error and its derivative
    (
    self.angular_error,
    self.angular_position_dot,
    self.angular_error_dot
    ) = Control.computeAngularErrorAndDerivative(
      self.odein,
      self.roll_ref,
      self.pitch_ref,
      self.angular_position_ref_dot
    )
        
    self.e_rot = self.odein.angular_velocity - self.omega_ref
    
    # Compute Inner Loop
    self.computeInnerLoop_mod()

    # Compute individual motor thrusts
    self.motor_thrusts = Control.computeMotorThrusts(self.fp, self.u1, self.u2, self.u3, self.u4)
  
  def ode(self, t, y):
    """
    Function called by RK4. Assumes `computeControlAlgorithm` was called
    at the beginning of the integration step to update internal state.
    """
    self.dy[0:2] = self.internal_state_differentiator_phi_ref_diff
    self.dy[2:4] = self.internal_state_differentiator_theta_ref_diff
    self.dy[4:7] = self.translational_position_error
    self.dy[7:10] = self.e_rot
    self.dy[10:13] = self.angular_error
    self.dy[13:16] = self.omega_ref - self.omega_cmd
    self.dy[16:19] = self.omega_ref_dot

    return np.array(self.dy)
  
  def computeOuterLoop(self):
    """
    Compute the translational control force vector (outer loop raw thrust)
    """
    velocity_error = self.odein.translational_velocity_in_I - self.odein.translational_velocity_in_I_user

    self.mu_tran_raw = (
      self.gains.mass_total_estimated * (
        - self.gains.KP_tran * self.translational_position_error
        - self.gains.KD_tran * velocity_error
        - self.gains.KI_tran * self.integral_position_tracking
        + self.odein.translational_acceleration_in_I_user
      )
    ).reshape(3, 1)

  def computeInnerLoop(self):
    """
    Computes control moments (u2, u3, u4) for the inner loop.
    """
    # Compute gyroscopic term
    gyro_term = np.cross(
      self.odein.angular_velocity.ravel(),
      (self.gains.I_matrix_estimated * self.odein.angular_velocity).ravel()
    ).reshape(3,1)

    # Compute feedback term
    feedback_term = self.gains.I_matrix_estimated * (
      - self.gains.KP_rot * self.angular_error
      - self.gains.KD_rot * self.angular_error_dot
      - self.gains.KI_rot * self.integral_angular_error
      + self.angular_position_ref_ddot
    ).reshape(3,1)
    
    self.Moment = gyro_term + feedback_term

    self.u2 = self.Moment[0].item()
    self.u3 = self.Moment[1].item()
    self.u4 = self.Moment[2].item()
    
  def computeOmegaCmdAndOmegaCmdDotInnerLoop(self):
    Jacobian_matrix = Control.computeJacobian(self.odein.roll, self.odein.pitch)

    Jacobian_matrix_dot = Control.computeJacobianDot(
      self.odein.roll,
      self.odein.pitch,
      self.angular_position_dot[0],
      self.angular_position_dot[1]
    )

    omega_cmd = Jacobian_matrix * (
      - self.gains.KP_rot * self.angular_error 
      - self.gains.KI_rot * self.integral_angular_error 
      + self.angular_position_ref_dot
    )
    
    omega_cmd_dot = (
      Jacobian_matrix_dot * (
        - self.gains.KP_rot * self.angular_error 
        - self.gains.KI_rot * self.integral_angular_error 
        + self.angular_position_ref_dot)
      + Jacobian_matrix * (
        - self.gains.KP_rot * self.angular_error_dot 
        - self.gains.KI_rot * self.angular_error 
        + self.angular_position_ref_ddot)
    )

    return omega_cmd, omega_cmd_dot
  
  def computeReferenceModelInnerLoop(self):
    omega_ref_dot = (
      - self.gains.K_P_omega_ref * (self.omega_ref - self.omega_cmd) 
      - self.gains.K_I_omega_ref * self.integral_e_omega_ref_cmd
      + self.omega_cmd_dot
    )

    return omega_ref_dot
  
  def computeReferenceCommandInputInnerLoop(self):
    r_rot = (
      self.gains.K_P_omega_ref * self.omega_cmd
      - self.gains.K_I_omega_ref * self.integral_e_omega_ref_cmd
      + self.omega_cmd_dot
    )

    return r_rot
  
  def computeMomentPIbaselineInnerLoop(self):
    Moment_baseline_PI = -self.fp.I_matrix_estimated * (
      self.gains.KP_rot_PI_baseline * self.e_rot
      + self.gains.KI_rot_PI_baseline * self.integral_e_rot 
      - self.omega_ref_dot
    )

    return Moment_baseline_PI
    
  def computeInnerLoop_mod(self):
    """
    Computes control moments (u2, u3, u4) for the inner loop.
    """
    
    (self.omega_cmd,
     self.omega_cmd_dot
    ) = self.computeOmegaCmdAndOmegaCmdDotInnerLoop()
    
    self.omega_ref_dot = self.computeReferenceModelInnerLoop()

    self.r_rot = self.computeReferenceCommandInputInnerLoop()

    self.Moment_baseline_PI = self.computeMomentPIbaselineInnerLoop()

    self.u2 = self.Moment_baseline_PI[0].item()
    self.u3 = self.Moment_baseline_PI[1].item()
    self.u4 = self.Moment_baseline_PI[2].item()