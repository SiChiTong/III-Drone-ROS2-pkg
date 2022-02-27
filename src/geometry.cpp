/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "geometry.h"

/*****************************************************************************/
// Function implementations
/*****************************************************************************/

rotation_matrix_t eulToR(orientation_t eul) {

    float cos_yaw = cos(eul[2]);
    float cos_pitch = cos(eul[1]);
    float cos_roll = cos(eul[0]);
    float sin_yaw = sin(eul[2]);
    float sin_pitch = sin(eul[1]);
    float sin_roll = sin(eul[0]);

    rotation_matrix_t mat;

    mat(0,0) = cos_pitch*cos_yaw;
    mat(0,1) = sin_roll*sin_pitch*cos_yaw-cos_roll*sin_yaw;
    mat(0,2) = cos_roll*sin_pitch*cos_yaw+sin_roll*sin_yaw;
    mat(1,0) = cos_pitch*sin_yaw;
    mat(1,1) = sin_roll*sin_pitch*sin_yaw+cos_roll*cos_yaw;
    mat(1,2) = cos_roll*sin_pitch*sin_yaw-sin_roll*cos_pitch;
    mat(2,0) = -sin_pitch;
    mat(2,1) = sin_roll*cos_pitch;
    mat(2,2) = cos_roll*cos_pitch;

    return mat;

}

vector_t rotateVector(rotation_matrix_t R, vector_t v) {

    vector_t ret_vec = R*v;

    return ret_vec;
}

point_t projectPointOnPlane(point_t point, plane_t plane) {

    line_t l = {
        .p = point,
        .v = plane.normal
    };

    float t = plane.normal.dot(plane.p-l.p);

    point_t proj_point = l.p + (point_t)(t*l.v);

    return proj_point;

}

orientation_t quatToEul(quat_t quat) {

    orientation_t eul(
        atan2(2*(quat[0]*quat[1] + quat[2]*quat[3]), 1-2*(quat[1]*quat[1] + quat[2]*quat[2])),
        asin(2*(quat[0]*quat[2] - quat[3]*quat[1])),
        atan2(2*(quat[0]*quat[3] + quat[1]*quat[2]), 1-2*(quat[2]*quat[2]+quat[3]*quat[3]))
    );

    return eul;

}

quat_t quatInv(quat_t quat) {

    quat_t ret_quat(quat[0], -quat[1], -quat[2], -quat[3]);

}

quat_t quatMultiply(quat_t quat1, quat_t quat2) {

    quat_t ret_quat(
        quat1[0]*quat2[0] - quat1[1]*quat2[1] - quat1[2]*quat2[2] - quat1[3]*quat2[3],
        quat1[0]*quat2[1] + quat1[1]*quat2[0] - quat1[2]*quat2[3] + quat1[3]*quat2[2],
        quat1[0]*quat2[2] + quat1[2]*quat2[0] + quat1[1]*quat2[3] - quat1[3]*quat2[1],
        quat1[0]*quat2[3] + quat1[3]*quat2[0] - quat1[1]*quat2[2] + quat1[2]*quat2[1]
    );

    return ret_quat;

}

rotation_matrix_t quatToMat(quat_t quat) {

    rotation_matrix_t mat;
    mat(0,0) = 1-2*quat[2]*quat[2]-2*quat[3]*quat[3];
    mat(0,1) = 2*quat[1]*quat[2]-2*quat[0]*quat[3];
    mat(0,2) = 2*quat[1]*quat[3]+2*quat[0]*quat[2];
    mat(1,0) = 2*quat[1]*quat[2]+2*quat[0]*quat[3];
    mat(1,1) = 1-2*quat[1]*quat[1]-2*quat[3]*quat[3];
    mat(1,2) = 2*quat[2]*quat[3]-2*quat[0]*quat[1];
    mat(2,0) = 2*quat[1]*quat[3]-2*quat[0]*quat[3];
    mat(2,1) = 2*quat[2]*quat[3]+2*quat[0]*quat[1];
    mat(2,2) = 1-2*quat[1]*quat[1]-2*quat[2]*quat[2];

    return mat;

}

quat_t matToQuat(rotation_matrix_t R) {

    float tr = R(0,0) + R(1,1) + R(2,2);

    float qw, qx, qy, qz;

    if (tr > 0) { 

        float S = sqrt(tr+1.0) * 2; // S=4*qw 
        qw = 0.25 * S;
        qx = (R(2,1) - R(1,2)) / S;
        qy = (R(0,2) - R(2,0)) / S; 
        qz = (R(1,0) - R(0,1)) / S; 

    } else if ((R(0,0) > R(1,1)) && (R(0,0) > R(2,2))) { 

        float S = sqrt(1.0 + R(0,0) - R(1,1) - R(2,2)) * 2; // S=4*qx 
        qw = (R(2,1) - R(1,2)) / S;
        qx = 0.25 * S;
        qy = (R(0,1) + R(1,0)) / S; 
        qz = (R(0,2) + R(2,0)) / S; 

    } else if (R(1,1) > R(2,2)) { 

        float S = sqrt(1.0 + R(1,1) - R(0,0) - R(2,2)) * 2; // S=4*qy
        qw = (R(0,2) - R(2,0)) / S;
        qx = (R(0,1) + R(1,0)) / S; 
        qy = 0.25 * S;
        qz = (R(1,2) + R(2,1)) / S; 

    } else { 

        float S = sqrt(1.0 + R(2,2) - R(0,0) - R(1,1)) * 2; // S=4*qz
        qw = (R(1,0) - R(0,1)) / S;
        qx = (R(0,2) + R(2,0)) / S;
        qy = (R(1,2) + R(2,1)) / S;
        qz = 0.25 * S;

    }

    quat_t quat(qw, qx, qy, qz);

    return quat;

}

transform_t getTransformMatrix(vector_t vec, quat_t quat) {

    transform_t T;

    rotation_matrix_t R = quatToMat(quat);

    for ( int i = 0; i < 3; i++) {

        for (int j = 0; j < 3; j++) {

            T(i,j) = R(i,j);
        }

        T(i,3) = vec(i);

    }

    T(3,0) = 0;
    T(3,1) = 0;
    T(3,2) = 0;
    T(3,3) = 1;

    return T;

}