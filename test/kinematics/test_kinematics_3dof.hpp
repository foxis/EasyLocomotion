#define KINEMATICS_3DOF_TESTS \
    _RUN_TEST(test_planar_kinematics_3dof_forward); \
    _RUN_TEST(test_planar_kinematics_3dof_forward1); \
    _RUN_TEST(test_planar_kinematics_3dof_forward2); \
    _RUN_TEST(test_planar_kinematics_3dof_inverse); \
    _RUN_TEST(test_planar_kinematics_3dof_inverse1); \
    _RUN_TEST(test_planar_kinematics_3dof_inverse2);


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


void test_planar_kinematics_3dof_forward() {
    PlanarKinematics3DOF k(joints_3dof, vol_3dof);

    real_t angles[] = {0, 0, 0};
    Vector3D p;
    
    k.forward(angles, p); 
    TEST_ASSERT_FLOAT_WITHIN(1e-6, p.x, 10 + 75 + 75);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, p.y, 0);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, p.z, 0);
}

void test_planar_kinematics_3dof_forward1() {
    PlanarKinematics3DOF k(joints_3dof, vol_3dof);

    real_t angles[] = {0, M_PI / 2, M_PI / 4};
    Vector3D p;
    
    k.forward(angles, p); 
    TEST_ASSERT_FLOAT_WITHIN(1e-3, p.x, -43.033);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, p.y, 128.033);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, p.z, 0);
}


void test_planar_kinematics_3dof_forward2() {
    PlanarKinematics3DOF k(joints_3dof, vol_3dof);

    real_t angles[] = {M_PI / 4, M_PI / 4, M_PI / 4};
    Vector3D p;
    
    k.forward(angles, p); 
    TEST_ASSERT_FLOAT_WITHIN(1e-3, p.x, 44.5711);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, p.y, 128.033);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, p.z, 44.5711);
}


void test_planar_kinematics_3dof_inverse() {
    PlanarKinematics3DOF k(joints_3dof, vol_3dof);

    real_t prev_angles[] = {0, 0, 0};
    real_t angles[] = {0, 0, 0};
    Vector3D target(40, 90, -10);
    Vector3D actual;
    
    bool done = k.inverse(target, prev_angles, angles, actual, 1, 10); 
    TEST_ASSERT_FLOAT_WITHIN(1e-3, angles[0], -.24497);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, angles[1], 2.11945);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, angles[2], -1.76532);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, (target - actual).magnitudeSqr(), 0);
    TEST_ASSERT_TRUE(done);
}

void test_planar_kinematics_3dof_inverse1() {
    PlanarKinematics3DOF k(joints_3dof, vol_3dof);

    real_t prev_angles[] = {0, 0, 0};
    real_t angles[] = {.13, -.3, -.1};
    real_t ik_angles[] = {0, 0, 0};
    Vector3D fw;
    Vector3D actual;

    k.forward(angles, fw);
    bool done = k.inverse(fw, prev_angles, ik_angles, actual, 1, 10); 

    TEST_ASSERT_FLOAT_WITHIN(1e-3, angles[0], ik_angles[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, angles[1], ik_angles[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, angles[2], ik_angles[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, (fw - actual).magnitudeSqr(), 0);
    TEST_ASSERT_TRUE(done);
}

void test_planar_kinematics_3dof_inverse2() {
    PlanarKinematics3DOF k(joints_3dof, vol_3dof);

    real_t prev_angles[] = {0, 0, 0};
    real_t angles[] = {-.7, .3, -.5};
    real_t ik_angles[] = {0, 0, 0};
    Vector3D fw;
    Vector3D actual;

    k.forward(angles, fw);
    bool done = k.inverse(fw, prev_angles, ik_angles, actual, 1, 10); 

    TEST_ASSERT_FLOAT_WITHIN(1e-3, angles[0], ik_angles[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, angles[1], ik_angles[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, angles[2], ik_angles[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, (fw - actual).magnitudeSqr(), 0);
    TEST_ASSERT_TRUE(done);
}
