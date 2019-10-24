#define MATRICES_4X4_TESTS \
    _RUN_TEST(test_matrices_4x4_det); \
    _RUN_TEST(test_matrices_4x4_inv);

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
