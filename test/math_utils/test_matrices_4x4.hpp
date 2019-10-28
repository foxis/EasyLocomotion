#define MATRICES_4X4_TESTS \
    _RUN_TEST(test_matrices_4x4_det); \
    _RUN_TEST(test_matrices_4x4_inv); \
    _RUN_TEST(test_matrices_4x4_mul_3d);

void test_matrices_4x4_det() {
    real_t data[] = {
        1, 2, 3, 1, 
        5, 1, 7, 3, 
        2, 1, 1, 0,
        0, 0, 1, 1
    };
    Matrix4x4 a(data);
    TEST_ASSERT_EQUAL(a.det(), 9);
}

void test_matrices_4x4_inv() {
    real_t data[] = {
        1, 2, 3, 1, 
        5, 1, 7, 3, 
        2, 1, 1, 0,
        0, 0, 1, 1
    };
    Matrix4x4 a(data);
    Matrix4x4 b;

    TEST_ASSERT_TRUE(a.inverse(b));

    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(0, 0), -1/3.0);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(0, 1), 0/3.0);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(0, 2), 2/3.0);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(0, 3), 1/3.0);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(1, 0), 1/3.0);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(1, 1), -1/3.0);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(1, 2), 2/3.0);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(1, 3), 2/3.0);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(2, 0), 1/3.0);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(2, 1), 1/3.0);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(2, 2), -1);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(2, 3), -4/3.0);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(3, 0), -1/3.0);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(3, 1), -1/3.0);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(3, 2), 1);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(3, 3), 7/3.0);
}

void test_matrices_4x4_mul_3d() {
    Matrix4x4 a(
        1, 0, 0, 1,
        0, 1, 0, 2,
        0, 0, 1, 3,
        0, 0, 0, 1
    );
    Vector3D v(2, 3, 4);
    Vector4D v4;

    a.mul((Vector4D)v, v4);

    TEST_ASSERT_FLOAT_WITHIN(1e-6, v4.x, 3);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, v4.y, 5);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, v4.z, 7);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, v4.w, 1);
}
