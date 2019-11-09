#define KINEMATICS_2DOF_TESTS \
    _RUN_TEST(test_planar_kinematics_2dof_forward); \
    _RUN_TEST(test_planar_kinematics_2dof_forward1); \
    _RUN_TEST(test_planar_kinematics_2dof_forward2); \
    _RUN_TEST(test_planar_kinematics_2dof_inverse); \
    _RUN_TEST(test_planar_kinematics_2dof_inverse1); \
    _RUN_TEST(test_planar_kinematics_2dof_inverse2);


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


void test_planar_kinematics_2dof_forward() {
    PlanarKinematics2DOF k(joints_2dof, vol_2dof);

    real_t angles[] = {0, 0};
    Vector3D p;
    
    k.forward(angles, p); 
    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.x, 85);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.y, 0);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.z, 0);
}

void test_planar_kinematics_2dof_forward1() {
    PlanarKinematics2DOF k(joints_2dof, vol_2dof);

    real_t angles[] = {0, M_PI / 2};
    Vector3D p;
    
    k.forward(angles, p); 
    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.x, 10);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.z, 75);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.y, 0);
}


void test_planar_kinematics_2dof_forward2() {
    PlanarKinematics2DOF k(joints_2dof, vol_2dof);

    real_t angles[] = {M_PI / 2, M_PI / 2};
    Vector3D p;
    
    k.forward(angles, p); 
    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.x, 0);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.z, 75);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, p.y, 10);
}


void test_planar_kinematics_2dof_inverse() {
    PlanarKinematics2DOF k(joints_2dof, vol_2dof);

    real_t prev_angles[] = {0, 0};
    real_t angles[] = {0, 0};
    Vector3D actual;
    Vector3D target(0, 10, 75);
    
    bool done = k.inverse(target, prev_angles, angles, actual, 1, 10); 
    TEST_ASSERT_FLOAT_WITHIN(1e-4, angles[0], M_PI / 2);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, angles[1], M_PI / 2);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, (target - actual).magnitudeSqr(), 0);
    TEST_ASSERT_TRUE(done);
}

void test_planar_kinematics_2dof_inverse1() {
    PlanarKinematics2DOF k(joints_2dof, vol_2dof);

    real_t prev_angles[] = {0, 0};
    real_t angles[] = {1.3, -.3};
    real_t ik_angles[] = {0, 0};
    Vector3D fw;
    Vector3D actual;

    k.forward(angles, fw);
    bool done = k.inverse(fw, prev_angles, ik_angles, actual, 1, 10); 

    TEST_ASSERT_FLOAT_WITHIN(1e-4, angles[0], ik_angles[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, angles[1], ik_angles[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, (fw - actual).magnitudeSqr(), 0);
    TEST_ASSERT_TRUE(done);
}

void test_planar_kinematics_2dof_inverse2() {
    PlanarKinematics2DOF k(joints_2dof, vol_2dof);

    real_t prev_angles[] = {0, 0};
    real_t angles[] = {-.3, 1.3};
    real_t ik_angles[] = {0, 0};
    Vector3D fw;
    Vector3D actual;

    k.forward(angles, fw);
    bool done = k.inverse(fw, prev_angles, ik_angles, actual, 1, 10); 

    TEST_ASSERT_FLOAT_WITHIN(1e-4, angles[0], ik_angles[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, angles[1], ik_angles[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, (fw - actual).magnitudeSqr(), 0);
    TEST_ASSERT_TRUE(done);
}
