import math
import numpy as np
# from numpy import linalg as LA
# import scipy
# from scipy import linalg
from acsl_pychrono.simulation.flight_params import FlightParams

class PIDGains:
  def __init__(self, flight_params: FlightParams):
    # General vehicle properties
    self.I_matrix_estimated = flight_params.I_matrix_estimated
    self.mass_total_estimated = flight_params.mass_total_estimated
    self.air_density_estimated = flight_params.air_density_estimated
    self.surface_area_estimated = flight_params.surface_area_estimated
    self.drag_coefficient_matrix_estimated = flight_params.drag_coefficient_matrix_estimated

    # Number of states to be integrated by RK4
    self.number_of_states = 19
    
    if flight_params.vehicle_config.vehicle_type == "thruststand_uav":
        # Length of the array vector that will be exported 
        self.size_DATA = 46
        
        # **Translational** PID parameters 
        self.KP_tran = np.matrix(1 * np.diag([5,5,6]))
        self.KD_tran = np.matrix(1 * np.diag([8,8,3]))
        self.KI_tran = np.matrix(1 * np.diag([1,1,1]))
        
        # **Rotational** baseline parameters
        self.KP_rot = np.matrix(1 * np.diag([50,50,3]))
        self.KI_rot = np.matrix(1 * np.diag([50,50,10]))

        # **Rotational** parameters for the PI baseline controller (Moment_baseline_PI)
        self.KP_rot_PI_baseline = np.matrix(1 * np.diag([10,50,5]))
        self.KI_rot_PI_baseline = np.matrix(1 * np.diag([10,10,10]))
        self.KD_rot_PI_baseline = np.matrix(0 * np.diag([1,1,1]))
        
        # **Rotational** reference trajectory gains
        self.K_P_omega_ref = np.matrix(1 * np.diag([30,50,5]))
        self.K_I_omega_ref = np.matrix(1 * np.diag([10,10,1]))
    elif flight_params.vehicle_config.vehicle_type == "x8copter":
        self.size_DATA = 50
        
        # **Translational** PID parameters 
        self.KP_tran = np.matrix(1 * np.diag([5,5,6]))
        self.KD_tran = np.matrix(1 * np.diag([8,8,3]))
        self.KI_tran = np.matrix(1 * np.diag([1,1,1]))
        
        # **Rotational** baseline parameters
        self.KP_rot = np.matrix(1 * np.diag([20,20,10]))
        
        self.KI_rot = np.matrix(1 * np.diag([10,10,10]))
        # **Rotational dynamics** parameters for the PI baseline controller (self.Moment_baseline_PI)       
        # self.KP_rot_PI_baseline = np.matrix(1 * np.diag([500,50,50]))
        self.KP_rot_PI_baseline = np.matrix(1 * np.diag([500,50,50]))
        self.KI_rot_PI_baseline = np.matrix(1 * np.diag([10,10,10]))
        self.KD_rot_PI_baseline = np.matrix(0 * np.diag([1,1,1])) # not needed and set to zero
        
        # **Reference dynamics** 
        self.K_P_omega_ref = np.matrix(1 * np.diag([10,10,10]))
        self.K_I_omega_ref = np.matrix(1 * np.diag([10,10,10]))
    
    # ----------------------------------------------------------------
    #                   Safety Mechanism Parameters
    # ----------------------------------------------------------------
    self.use_safety_mechanism = False
    
    # Mu - sphere intersection
    self.sphereEpsilon = 1e-2
    self.maximumThrust = 85 # [N] 85
    
    # Mu - elliptic cone intersection
    self.EllipticConeEpsilon = 1e-2
    self.maximumRollAngle = math.radians(32) # [rad] 25
    self.maximumPitchAngle = math.radians(32) # [rad] 25
    
    # Mu - plane intersection
    self.planeEpsilon = 1e-2
    self.alphaPlane = 0.95 # [-] coefficient for setting the 'height' of the bottom plane. Must be >0 and <1.
