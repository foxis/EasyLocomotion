#define KINEMATICS_TESTS \
    KINEMATICS_2DOF_TESTS; \
    KINEMATICS_3DOF_TESTS; \
    KINEMATICS_DH_TESTS; \
    KINEMATICS_BODY_TESTS; \


#include "kinematics/PlanarKinematics.h"
#include "kinematics/DenavitHartenbergKinematics.h"

#include "test_kinematics_2dof.hpp"
#include "test_kinematics_3dof.hpp"
#include "test_kinematics_dh.hpp"
#include "test_kinematics_body.hpp"
