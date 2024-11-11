import numpy as np
from properties import *

'''
take input of either:
disturbance torques (vector)
or
target angular velocity (vector)

compute the required reaction

compute the possible reaction wheel responses
find the most optimal solution

compute the possible RCS thruster responses
find the most optimal solution
'''


def desired_velocity():
    print('Give desired angular velocities of the SC [rad/s]')
    omega_x = float(input('around SC x-axis: '))
    omega_y = float(input('around SC y-axis: '))
    omega_z = float(input('around SC z-axis: '))
    return np.array([omega_x, omega_y, omega_z])


def desired_acceleration():
    print('Give desired angular accelerations of the SC [rad/s^2]')
    alpha_x = float(input('around SC x-axis: '))
    alpha_y = float(input('around SC y-axis: '))
    alpha_z = float(input('around SC z-axis: '))
    return np.array([alpha_x, alpha_y, alpha_z])


def reaction_wheels(sc_omega):
    """Equation to solve in the form Ur=m, where m is the momentum vector of the SC, r is the momenta of the reaction wheels
     and U is a matrix with reaction wheel directions. We are solving for r"""

    # Preparing the vector m
    m = sc_inertia @ sc_omega
    m_scaled = m / rw_inertia

    # Preparing the matrix U
    U = np.vstack(rw_directions).T

    # Solving the system
    R_particular, residuals, rank, s = np.linalg.lstsq(U, m_scaled, rcond=None)
    U_svd, S, Vt = np.linalg.svd(U)
    v_null = Vt.T[:, len(S):]

    print('R_particular [rad/s]: ')
    print(R_particular)
    print('')

    if np.any(np.abs(R_particular) > omega_max):
        print(f'\033[31mReaction wheels oversaturated\033[0m\n'
              'Scaling down to maximum saturation [rad/s]')
        R_norm = R_particular / np.max(abs(R_particular)) * omega_max
        print(R_norm)
        possible_omega = np.linalg.inv(sc_inertia) @ U @ R_norm * rw_inertia
        print('\nThe achievable angular velocity vector is instead [rad/s]:')
        print(possible_omega)
        ratio = 100 * np.mean(np.divide(possible_omega, sc_omega, out=np.zeros_like(possible_omega), where=(sc_omega != 0)))
        print(f'Which is about {ratio:.2f}% of the input value')
        print('')


def thrusters_acceleration(sc_alpha):
    # Preparing the vector tau
    tau = sc_inertia @ sc_alpha

    # solving for the vector of thrusts
    T_particular, residuals, rank, s = np.linalg.lstsq(rcs_torque_matrix, tau, rcond=None)

    T = T_particular.copy()
    # correct for negative thrusts by utilising colinear thruster pairs
    for i, t in enumerate(T_particular):
        if t < 0:
            couple = thruster_couple[i+1]
            T[couple - 1] += -t
            T[i] = 0

    print('\nThe thrusts of all thrusters [N]:')
    print(T)
    print('')

    if np.any(T > rcs_thrust_max):
        print(f'\033[31mAcceleration unfeasible due to excessive thrust\033[0m \n'
              'Scaling back to nominal thrusts [N]:')

        T_norm = T/np.max(T) * rcs_thrust_max
        print(T_norm)
        possible_alpha = np.linalg.inv(sc_inertia) @ rcs_torque_matrix @ T_norm
        print('\nThe achievable angular acceleration vector is instead [rad/s^2]:')
        print(possible_alpha)
        ratio = 100 * np.mean(np.divide(possible_alpha, sc_alpha, out=np.zeros_like(possible_alpha), where=(sc_alpha != 0)))
        print(f'Which is about {ratio:.2f}% of the input value')
        print('')


def thrusters_velocity(sc_omega):
    m = sc_inertia @ sc_omega
    # assume initial firing time = 1 [s]

    # solving for the vector of thrusts
    T_particular, residuals, rank, s = np.linalg.lstsq(rcs_torque_matrix, m, rcond=None)
    T = T_particular.copy()
    # correct for negative thrusts by utilising colinear thruster pairs
    for i, t in enumerate(T_particular):
        if t < 0:
            couple = thruster_couple[i+1]
            T[couple - 1] += -t
            T[i] = 0

    # utilise maximum thrust
    T_norm = T / np.max(T) * rcs_thrust_max
    possible_alpha = np.linalg.inv(sc_inertia) @ rcs_torque_matrix @ T_norm
    # compute the burn time
    ratio_vector = np.divide(sc_omega, possible_alpha, out=np.zeros_like(sc_omega), where=(possible_alpha != 0))
    burn_time = np.mean(ratio_vector[ratio_vector != 0])  # technically, np.mean is unnecessary since all nonzero values will be equal
    print(f'\nTo achieve such omega, the thrusters shall burn for {burn_time:.2f} [s] with the following thrusts [N]:')
    print(T_norm)
    print('')


def main():
    while True:
        print('===========================================')
        used_system = input('Select reaction method:\n T hrusters    R eaction wheels\n')
        if used_system.lower() == 't':
            simulation_type = input('Select simulation target:\n angular A cceleration    angular V elocity\n')
            if simulation_type.lower() == 'a':
                sc_alpha = desired_acceleration()
                thrusters_acceleration(sc_alpha)
            elif simulation_type.lower() == 'v':
                sc_omega = desired_velocity()
                thrusters_velocity(sc_omega)
            else:
                print('Invalid command')
        elif used_system.lower() == 'r':
            sc_omega = desired_velocity()
            reaction_wheels(sc_omega)
        else:
            print('Invalid command')


if __name__ == '__main__':
    main()
