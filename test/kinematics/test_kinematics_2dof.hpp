#define KINEMATICS_3DOF_TESTS \
    _RUN_TEST(test_planar_kinematics_2dof_forward); \
    _RUN_TEST(test_planar_kinematics_2dof_forward1); \
    _RUN_TEST(test_planar_kinematics_2dof_forward2); 


PlanarJoint_t joints_2dof[] = {
    {
        .constraints = {
            .min = -1, 
            .max = 1
        }, 
        .length = 10 
    },
    {
        .constraints = {-.5, 1}, 
        .length = 75
    },
};
ConstraintVolume vol_2dof(Vector3D(-10, -10, -10), Vector3D(10, 10, 10));


void test_planar_kinematics_2dof_forward() {
    PlanarKinematics2DOF k(joints_2dof, vol_2dof);

    real_t angles[] = {0, 0};
    Vector3D p;
    
    k.forward(angles, p); 
    TEST_ASSERT_FLOAT_WITHIN(1e-6, p.x, 85);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, p.y, 0);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, p.z, 0);
}

void test_planar_kinematics_2dof_forward1() {
    PlanarKinematics2DOF k(joints_2dof, vol_2dof);

    real_t angles[] = {0, M_PI / 2};
    Vector3D p;
    
    k.forward(angles, p); 
    TEST_ASSERT_FLOAT_WITHIN(1e-6, p.x, 10);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, p.y, 75);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, p.z, 0);
}


void test_planar_kinematics_2dof_forward2() {
    PlanarKinematics2DOF k(joints_2dof, vol_2dof);

    real_t angles[] = {M_PI / 2, M_PI / 2};
    Vector3D p;
    
    k.forward(angles, p); 
    TEST_ASSERT_FLOAT_WITHIN(1e-6, p.x, 0);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, p.y, 75);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, p.z, 10);
}
