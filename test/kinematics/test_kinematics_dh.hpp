#define KINEMATICS_DH_TESTS \
    _RUN_TEST(test_dh_kinematics_2dof_forward); \
    _RUN_TEST(test_dh_kinematics_3dof_forward); \


void test_dh_kinematics_2dof_forward() {
    PlanarJoint_t joints_2dof[] = {
        {
            .constraints = {
                .min = -M_PI, 
                .max = M_PI
            }, 
            .length = 10 
        },
        {
            .constraints = {-M_PI, M_PI}, 
            .length = 75
        },
    };
    ConstraintVolume vol_2dof(Vector3D(-10, -10, -10), Vector3D(10, 10, 10));

    real_t dh_params_arr[] = {
        0, M_PI / 2.0, 10, 0, 0,
        0, 0, 75, 0, 0,
    };
    ConstraintSegment constraints[2] = {
        {.min=-M_PI, .max=M_PI}, 
        {.min=-M_PI, .max=M_PI}, 
    };

    PlanarKinematics2DOF k(joints_2dof, vol_2dof);
    _DenavitHartenbergKinematics<real_t, 2> dhk(dh_params_arr, constraints, vol_2dof);

    real_t angles[] = {0, 0};
    real_t angles1[] = {.7, .3};
    real_t angles2[] = {-.7, -.1};
    Vector3D p;
    Vector3D pdh;
    
    k.forward(angles, p); 
    dhk.forward(angles, pdh);

    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.x, pdh.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.y, pdh.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.z, pdh.z);

    k.forward(angles1, p); 
    dhk.forward(angles1, pdh);

    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.x, pdh.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.y, pdh.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.z, pdh.z);

    k.forward(angles2, p); 
    dhk.forward(angles2, pdh);

    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.x, pdh.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.y, pdh.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.z, pdh.z);
}

void test_dh_kinematics_3dof_forward() {
    PlanarJoint_t joints_3dof[] = {
        {
            .constraints = {
                .min = -M_PI, 
                .max = M_PI
            }, 
            .length = 10 
        },
        {
            .constraints = {-M_PI, M_PI}, 
            .length = 75
        },
        {
            .constraints = {-M_PI, M_PI}, 
            .length = 75
        },
    };
    ConstraintVolume vol_3dof(Vector3D(-10, -10, -10), Vector3D(10, 10, 10));
    real_t dh_params_arr[] = {
        0, M_PI / 2, 10, 0, 0,
        0, 0, 75, 0, 0,
        0, 0, 75, 0, 0,
    };
    ConstraintSegment constraints[] = {
        {.min=-M_PI, .max=M_PI}, 
        {.min=-M_PI, .max=M_PI}, 
        {.min=-M_PI, .max=M_PI}, 
    };

    PlanarKinematics3DOF k(joints_3dof, vol_3dof);
    _DenavitHartenbergKinematics<real_t, 3> dhk(dh_params_arr, constraints, vol_2dof);

    real_t angles[] = {0, 0, 0};
    real_t angles1[] = {.7, .3, -.1};
    real_t angles2[] = {-.1, .3, -.4};
    Vector3D p;
    Vector3D pdh;

    k.forward(angles, p); 
    dhk.forward(angles, pdh);

    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.x, pdh.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.y, pdh.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.z, pdh.z);

    k.forward(angles1, p); 
    dhk.forward(angles1, pdh);

    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.x, pdh.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.y, pdh.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.z, pdh.z);

    k.forward(angles2, p); 
    dhk.forward(angles2, pdh);

    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.x, pdh.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.y, pdh.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.z, pdh.z);
}