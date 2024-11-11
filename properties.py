import numpy as np
import math as m


def componentwise_cross(first, second):
    output = []
    for i, j in zip(first, second):
        ixj = np.cross(i, -j)
        output.append(ixj)
    return np.vstack(output)


"""Spacecraft Properties"""
Ixx, Iyy, Izz = 517, 283, 329  # TODO GIVE THE CORRECT INERTIAS
sc_inertia = np.diag([Ixx, Iyy, Izz])  # kgm^2
sc_mass = 1661  # kg TODO GIVE THE CORRECT MASS

"""RCS Properties"""
rcs_thrust_max = 12.5  # N
rcs_locations_list = (np.array([0.    , -.9786, 1.25 ]),
                      np.array([.565  , -.9786, 0.   ]),
                      np.array([0.    , -.9786, -1.25]),
                      np.array([-.565 , -.9786, 0.   ]),
                      np.array([.8475 , .4893 , 1.25 ]),
                      np.array([.565  , .9786 , 0.   ]),
                      np.array([.8475 , .4893 , -1.25]),
                      np.array([1.13  , 0.    , 0.   ]),
                      np.array([-.8475, .4893 , 1.25 ]),
                      np.array([-1.13 , 0.    , 0.   ]),
                      np.array([-.8475, .4893 , -1.25]),
                      np.array([-.565 , .9786 , 0.   ]))  # m

rcs_directions_list = (np.array([0. , 0.   , 1. ]),
                       np.array([1. , 0.   , 0. ]),
                       np.array([0. , 0.   , -1.]),
                       np.array([-1., 0.   , 0. ]),
                       np.array([0. , 0.   , 1. ]),
                       np.array([-.5, .866 , 0. ]),
                       np.array([0. , 0.   , -1.]),
                       np.array([.5 , -.866, 0. ]),
                       np.array([0. , 0.   , 1. ]),
                       np.array([-.5, -.866, 0. ]),
                       np.array([0. , 0.   , -1.]),
                       np.array([.5 , .866 , 0. ]))  # -

rcs_torque_matrix = componentwise_cross(rcs_locations_list, rcs_directions_list).T

thruster_couple = (None, 3, 4, 1, 2, 7, 8, 5, 6, 11, 12, 9, 10)
# The None is there, so I don't have to worry about the 0-index

"""Reaction Wheel Properties"""
rw_mass = 5.2  # kg
momentum_at_3000rpm = 4  # Nms

rw_inertia = momentum_at_3000rpm / (3000 * 2 * np.pi / 60)  # kgm^2

rw_directions = (np.array([0., 0., 1.]),
                 np.array([-np.sqrt(2) / 3   , -np.sqrt(6) / 3, -1 / 3]),
                 np.array([-np.sqrt(2) / 3   , np.sqrt(6) / 3 , -1 / 3]),
                 np.array([np.sqrt(2) * 2 / 3, 0.             , -1 / 3]))  # -

omega_max = 4500 * 2 * np.pi / 60  # rad/s
momentum_min = 0
momentum_max = omega_max * rw_inertia



