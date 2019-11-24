#define KINEMATICS_BODY_TESTS \
    _RUN_TEST(test_kinematics_body); \
    _RUN_TEST(test_kinematics_body_pos); \

void test_kinematics_body() {
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
    ConstraintVolume vol_2dof(Vector3D(-10+10+75, -10, -10), Vector3D(10+10+75, 10, 10));
    PlanarKinematics2DOF limb(joints_2dof, vol_2dof);

    LimbKinematicsModel* limbs[] = {
        &limb,
        &limb,
    };
    LimbConfig_t limb_config[] = {
        {
            .displacement=Vector3D(10, -10, 0),
            .orientation=Vector3D(0, 0, 0),
        },
        {
            .displacement=Vector3D(-10, 10, 0),
            .orientation=Vector3D(0, 0, M_PI),
        },
    };
    Vector3D position(0.0);
    Vector3D orientation(0.0);
    ConstraintVolume working_space(-30, -30, -60, 30, 30, 60);

	BodyModel<2> body(limbs, limb_config, working_space, position, orientation);

    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_working_space[0].min.x, 0+10+75);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_working_space[0].min.y, -20);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_working_space[0].min.z, -10);

    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_working_space[0].max.x, 20+10+75);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_working_space[0].max.y, 0);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_working_space[0].max.z, 10);

    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_working_space[1].min.x, -20-10-75);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_working_space[1].min.y, 0);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_working_space[1].min.z, -10);

    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_working_space[1].max.x, 0-10-75);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_working_space[1].max.y, 20);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_working_space[1].max.z, 10);

    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[0].x, 10+10+75);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[0].y, -10);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[0].z, 0);

    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[1].x, -10-10-75);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[1].y, 10);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[1].z, 0);

    body.current_joints[1] = -M_PI/4;
    body.current_joints[3] = -M_PI/4;
    body.forward_limbs();

    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[0].x, 73.033005);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[0].y, -10);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[0].z, -53.033009);

    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[1].x, -73.033005);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[1].y, 10);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[1].z, -53.033009);

    body.limb_pos[0] = Vector3D(73, -8, -53);
    body.limb_pos[1] = Vector3D(-73, 12, -53);
    body.inverse_limbs(1e-5, 10);

    body.forward_limbs();

    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[0].x, body.actual_limb_pos[0].x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[0].y, body.actual_limb_pos[0].y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[0].z, body.actual_limb_pos[0].z);

    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[1].x, body.actual_limb_pos[1].x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[1].y, body.actual_limb_pos[1].y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[1].z, body.actual_limb_pos[1].z);

    body.set_position(Vector3D(0, 2, 53));
    bool supports[] = {true, true};
    body.inverse(supports, 1e-5, 10);

    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[0].x, 95);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[0].y, -12);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[0].z, -53);

    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[1].x, -95);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[1].y, 8);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[1].z, -53);

    body.set_transform(Vector3D(0, 2, 53), Vector3D(0, 0, M_PI/20.0));
    body.inverse(supports, 1e-5, 10);

    print_vector(body.limb_pos[0], "body inv limb 0");
    print_vector(body.actual_limb_pos[0], "body inv limb actual 0");
    print_arr<real_t, 2>(body.target_joints, "target joints 0");

    print_vector(body.limb_pos[1], "body inv limb 1");
    print_vector(body.actual_limb_pos[1], "body inv limb actual 1");
    print_arr<real_t, 2>(body.target_joints + 2, "target joints 1");

    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[0].x, 91.953178);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[0].y, -26.713535);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[0].z, -53);

    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[1].x, -92.578911);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[1].y, 22.762775);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[1].z, -53);
}

void test_kinematics_body_pos() {
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
    ConstraintVolume vol_2dof(Vector3D(-10+10+75, -10, -10), Vector3D(10+10+75, 10, 10));
    PlanarKinematics2DOF limb(joints_2dof, vol_2dof);

    LimbKinematicsModel* limbs[] = {
        &limb,
        &limb,
    };
    LimbConfig_t limb_config[] = {
        {
            .displacement=Vector3D(10, -10, 0),
            .orientation=Vector3D(0, 0, 0),
        },
        {
            .displacement=Vector3D(-10, 10, 0),
            .orientation=Vector3D(0, 0, M_PI),
        },
    };
    Vector3D position(0, 2, 53);
    Vector3D orientation(0, 0, M_PI/20.0);
    ConstraintVolume working_space(-30, -30, -60, 30, 30, 60);

	BodyModel<2> body(limbs, limb_config, working_space, position, orientation);

    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[0].x, 91.953178);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[0].y, -26.713535);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[0].z, -53);

    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[1].x, -92.578911);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[1].y, 22.762775);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, body.limb_pos[1].z, -53);
}