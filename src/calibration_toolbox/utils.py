# Utility functions
# Author: Hongtao Wu
# Johns Hopkins University

import numpy as np
import yaml
from yaml import CLoader

def quat2rotm(quat):
    """
    Quaternion to rotation matrix.
    
    Args:
    - quat (4, numpy array): quaternion w, x, y, z
    Returns:
    - rotm: (3x3 numpy array): rotation matrix
    """
    w = quat[0]
    x = quat[1]
    y = quat[2]
    z = quat[3]

    s = w*w + x*x + y*y + z*z

    rotm = np.array([[1-2*(y*y+z*z)/s, 2*(x*y-z*w)/s,   2*(x*z+y*w)/s  ],
                     [2*(x*y+z*w)/s,   1-2*(x*x+z*z)/s, 2*(y*z-x*w)/s  ],
                     [2*(x*z-y*w)/s,   2*(y*z+x*w)/s,   1-2*(x*x+y*y)/s]
    ])

    return rotm

def make_rigid_transformation(pos, rotm):
    """
    Rigid transformation from position and orientation.
    Args:
    - pos (3, numpy array): translation
    - rotm (3x3 numpy array): orientation in rotation matrix
    Returns:
    - homo_mat (4x4 numpy array): homogenenous transformation matrix
    """
    homo_mat = np.c_[rotm, np.reshape(pos, (3, 1))]
    homo_mat = np.r_[homo_mat, [[0, 0, 0, 1]]]
    
    return homo_mat

def pose_inv(pose):
    """
    Inverse of a homogenenous transformation.
    Args:
    - pose (4x4 numpy array)
    Return:
    - inv_pose (4x4 numpy array)
    """
    R = pose[:3, :3]
    t = pose[:3, 3]

    inv_R = R.T
    inv_t = - np.dot(inv_R, t)

    inv_pose = np.c_[inv_R, np.transpose(inv_t)]
    inv_pose = np.r_[inv_pose, [[0, 0, 0, 1]]]

    return inv_pose


def get_mat_log(R):
  """
  Get the log(R) of the rotation matrix R.
  
  Args:
  - R (3x3 numpy array): rotation matrix
  Returns:
  - w (3, numpy array): log(R)
  """
  theta = np.arccos((np.trace(R) - 1) / 2)
  w_hat = (R - R.T) * theta / (2 * np.sin(theta))  # Skew symmetric matrix
  w = np.array([w_hat[2, 1], w_hat[0, 2], w_hat[1, 0]])  # [w1, w2, w3]

  return w


def rotm2quat(R):
    """
    Get the quaternion from rotation matrix.
    
    Args:
    - R (3x3 numpy array): rotation matrix
    Return:
    - q (4, numpy array): quaternion, w, x, y, z
    """
    w = get_mat_log(R)
    theta = np.linalg.norm(w)

    if theta < 0.001:
        q = np.array([0, 0, 0, 1])
        return q

    axis = w / theta

    q = np.sin(theta/2) * axis
    q = np.r_[np.cos(theta/2), q]

    return q

def rotm2angle(R):
    # From: euclideanspace.com

    epsilon = 0.01 # Margin to allow for rounding errors
    epsilon2 = 0.1 # Margin to distinguish between 0 and 180 degrees

    assert(isRotm(R))

    if ((abs(R[0][1]-R[1][0])< epsilon) and (abs(R[0][2]-R[2][0])< epsilon) and (abs(R[1][2]-R[2][1])< epsilon)):
        # Singularity found
        # First check for identity matrix which must have +1 for all terms in leading diagonaland zero in other terms
        if ((abs(R[0][1]+R[1][0]) < epsilon2) and (abs(R[0][2]+R[2][0]) < epsilon2) and (abs(R[1][2]+R[2][1]) < epsilon2) and (abs(R[0][0]+R[1][1]+R[2][2]-3) < epsilon2)):
            # this singularity is identity matrix so angle = 0
            return [0,1,0,0] # zero angle, arbitrary axis

        # Otherwise this singularity is angle = 180
        angle = np.pi
        xx = (R[0][0]+1)/2
        yy = (R[1][1]+1)/2
        zz = (R[2][2]+1)/2
        xy = (R[0][1]+R[1][0])/4
        xz = (R[0][2]+R[2][0])/4
        yz = (R[1][2]+R[2][1])/4
        if ((xx > yy) and (xx > zz)): # R[0][0] is the largest diagonal term
            if (xx< epsilon):
                x = 0
                y = 0.7071
                z = 0.7071
            else:
                x = np.sqrt(xx)
                y = xy/x
                z = xz/x
        elif (yy > zz): # R[1][1] is the largest diagonal term
            if (yy< epsilon):
                x = 0.7071
                y = 0
                z = 0.7071
            else:
                y = np.sqrt(yy)
                x = xy/y
                z = yz/y
        else: # R[2][2] is the largest diagonal term so base result on this
            if (zz< epsilon):
                x = 0.7071
                y = 0.7071
                z = 0
            else:
                z = np.sqrt(zz)
                x = xz/z
                y = yz/z
        return [angle,x,y,z] # Return 180 deg rotation

    # As we have reached here there are no singularities so we can handle normally
    s = np.sqrt((R[2][1] - R[1][2])*(R[2][1] - R[1][2]) + (R[0][2] - R[2][0])*(R[0][2] - R[2][0]) + (R[1][0] - R[0][1])*(R[1][0] - R[0][1])) # used to normalise
    if (abs(s) < 0.001):
        s = 1 

    # Prevent divide by zero, should not happen if matrix is orthogonal and should be
    # Caught by singularity test above, but I've left it in just in case
    angle = np.arccos(( R[0][0] + R[1][1] + R[2][2] - 1)/2)
    x = (R[2][1] - R[1][2])/s
    y = (R[0][2] - R[2][0])/s
    z = (R[1][0] - R[0][1])/s

    aa = angle * np.array([x, y, z])

    return aa

def isRotm(R) :
    # Checks if a matrix is a valid rotation matrix.
    # Forked from Andy Zeng
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def getDictFromYamlFilename(filename):
    """
    Read data from a YAML files
    """
    return yaml.load(file(filename), Loader=CLoader)