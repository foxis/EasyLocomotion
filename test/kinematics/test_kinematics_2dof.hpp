#define KINEMATICS_3DOF_TESTS


void test_planar_kinematics_2dof_forward() {
    JointConfig_t joints[] = {
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
    ConstraintVolume vol(Vector3D(-10, -10, -10), Vector3D(10, 10, 10));
    PlanarKinematics2DOF k(joints, vol);

    real_t angles[] = {0, 0};
    Vector3D p;
    
    k.forward(angles, p); 
    TEST_ASSERT_FLOAT_WITHIN(p.x, 0);
    TEST_ASSERT_FLOAT_WITHIN(p.y, 0);
    TEST_ASSERT_FLOAT_WITHIN(p.y, 0);
}

