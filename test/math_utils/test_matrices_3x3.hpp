#define MATRICES_3X3_TESTS \
    _RUN_TEST(test_matrices_3x3_constructor); \
    _RUN_TEST(test_matrices_3x3_transpose); \
    _RUN_TEST(test_matrices_3x3_transpose_inplace); \
    _RUN_TEST(test_matrices_3x3_add); \
    _RUN_TEST(test_matrices_3x3_sub); \
    _RUN_TEST(test_matrices_3x3_mul); \
    _RUN_TEST(test_matrices_3x3_mul_mat); \
    _RUN_TEST(test_matrices_3x3_mul_vec); \
    _RUN_TEST(test_matrices_3x3_div); \
    _RUN_TEST(test_matrices_3x3_det); \
    _RUN_TEST(test_matrices_3x3_inv);


void test_matrices_3x3_constructor() {
    real_t data[] = {0, 1, 1, 2, 4, 5, 6, 7, 8};
    Matrix3x3 a;
    Matrix3x3 b(1);
    Matrix3x3 c(1, 0, 2, 1, 1, 2, 3, 4, 5);
    Matrix3x3 d(data);
    Matrix3x3 e(c);
    Matrix3x3 f;

    a.eye();
    f = d;

    TEST_ASSERT_EQUAL(a.val(0, 0), 1);
    TEST_ASSERT_EQUAL(a.val(0, 1), 0);
    TEST_ASSERT_EQUAL(a.val(0, 2), 0);
    TEST_ASSERT_EQUAL(a.val(1, 0), 0);
    TEST_ASSERT_EQUAL(a.val(1, 1), 1);
    TEST_ASSERT_EQUAL(a.val(1, 2), 0);
    TEST_ASSERT_EQUAL(a.val(2, 0), 0);
    TEST_ASSERT_EQUAL(a.val(2, 1), 0);
    TEST_ASSERT_EQUAL(a.val(2, 2), 1);

    TEST_ASSERT_EQUAL(b.val(0, 0), 1);
    TEST_ASSERT_EQUAL(b.val(0, 1), 1);
    TEST_ASSERT_EQUAL(b.val(0, 2), 1);
    TEST_ASSERT_EQUAL(b.val(1, 0), 1);
    TEST_ASSERT_EQUAL(b.val(1, 1), 1);
    TEST_ASSERT_EQUAL(b.val(1, 2), 1);
    TEST_ASSERT_EQUAL(b.val(2, 0), 1);
    TEST_ASSERT_EQUAL(b.val(2, 1), 1);
    TEST_ASSERT_EQUAL(b.val(2, 2), 1);

    TEST_ASSERT_EQUAL(c.val(0, 0), 1);
    TEST_ASSERT_EQUAL(c.val(0, 1), 0);
    TEST_ASSERT_EQUAL(c.val(0, 2), 2);
    TEST_ASSERT_EQUAL(c.val(1, 0), 1);
    TEST_ASSERT_EQUAL(c.val(1, 1), 1);
    TEST_ASSERT_EQUAL(c.val(1, 2), 2);
    TEST_ASSERT_EQUAL(c.val(2, 0), 3);
    TEST_ASSERT_EQUAL(c.val(2, 1), 4);
    TEST_ASSERT_EQUAL(c.val(2, 2), 5);

    TEST_ASSERT_EQUAL(d.val(0, 0), 0);
    TEST_ASSERT_EQUAL(d.val(0, 1), 1);
    TEST_ASSERT_EQUAL(d.val(0, 2), 1);
    TEST_ASSERT_EQUAL(d.val(1, 0), 2);
    TEST_ASSERT_EQUAL(d.val(1, 1), 4);
    TEST_ASSERT_EQUAL(d.val(1, 2), 5);
    TEST_ASSERT_EQUAL(d.val(2, 0), 6);
    TEST_ASSERT_EQUAL(d.val(2, 1), 7);
    TEST_ASSERT_EQUAL(d.val(2, 2), 8);

    TEST_ASSERT_EQUAL(e.val(0, 0), 1);
    TEST_ASSERT_EQUAL(e.val(0, 1), 0);
    TEST_ASSERT_EQUAL(e.val(0, 2), 2);
    TEST_ASSERT_EQUAL(e.val(1, 0), 1);
    TEST_ASSERT_EQUAL(e.val(1, 1), 1);
    TEST_ASSERT_EQUAL(e.val(1, 2), 2);
    TEST_ASSERT_EQUAL(e.val(2, 0), 3);
    TEST_ASSERT_EQUAL(e.val(2, 1), 4);
    TEST_ASSERT_EQUAL(e.val(2, 2), 5);

    TEST_ASSERT_EQUAL(f.val(0, 0), 0);
    TEST_ASSERT_EQUAL(f.val(0, 1), 1);
    TEST_ASSERT_EQUAL(f.val(0, 2), 1);
    TEST_ASSERT_EQUAL(f.val(1, 0), 2);
    TEST_ASSERT_EQUAL(f.val(1, 1), 4);
    TEST_ASSERT_EQUAL(f.val(1, 2), 5);
    TEST_ASSERT_EQUAL(f.val(2, 0), 6);
    TEST_ASSERT_EQUAL(f.val(2, 1), 7);
    TEST_ASSERT_EQUAL(f.val(2, 2), 8);
}

void test_matrices_3x3_transpose() {
    Matrix3x3 a(1, 2, 3, 4, 5, 6, 7, 8, 9);
    Matrix3x3 b;

    a.transpose(b);
    TEST_ASSERT_EQUAL(b.val(0, 0), 1);
    TEST_ASSERT_EQUAL(b.val(0, 1), 4);
    TEST_ASSERT_EQUAL(b.val(0, 2), 7);
    TEST_ASSERT_EQUAL(b.val(1, 0), 2);
    TEST_ASSERT_EQUAL(b.val(1, 1), 5);
    TEST_ASSERT_EQUAL(b.val(1, 2), 8);
    TEST_ASSERT_EQUAL(b.val(2, 0), 3);
    TEST_ASSERT_EQUAL(b.val(2, 1), 6);
    TEST_ASSERT_EQUAL(b.val(2, 2), 9);
}

void test_matrices_3x3_transpose_inplace() {
    Matrix3x3 a(1, 2, 3, 4, 5, 6, 7, 8, 9);

    a.transpose();
    TEST_ASSERT_EQUAL(a.val(0, 0), 1);
    TEST_ASSERT_EQUAL(a.val(0, 1), 4);
    TEST_ASSERT_EQUAL(a.val(0, 2), 7);
    TEST_ASSERT_EQUAL(a.val(1, 0), 2);
    TEST_ASSERT_EQUAL(a.val(1, 1), 5);
    TEST_ASSERT_EQUAL(a.val(1, 2), 8);
    TEST_ASSERT_EQUAL(a.val(2, 0), 3);
    TEST_ASSERT_EQUAL(a.val(2, 1), 6);
    TEST_ASSERT_EQUAL(a.val(2, 2), 9);
}

void test_matrices_3x3_add() {
    Matrix3x3 a(1, 2, 3, 4, 5, 6, 7, 8, 9);
    Matrix3x3 b(0, 1, 1, 0, 2, 2, 3, 3, 3);
    Matrix3x3 c;

    c = a + b;

    TEST_ASSERT_EQUAL(c.val(0, 0), 1);
    TEST_ASSERT_EQUAL(c.val(0, 1), 3);
    TEST_ASSERT_EQUAL(c.val(0, 2), 4);
    TEST_ASSERT_EQUAL(c.val(1, 0), 4);
    TEST_ASSERT_EQUAL(c.val(1, 1), 7);
    TEST_ASSERT_EQUAL(c.val(1, 2), 8);
    TEST_ASSERT_EQUAL(c.val(2, 0), 10);
    TEST_ASSERT_EQUAL(c.val(2, 1), 11);
    TEST_ASSERT_EQUAL(c.val(2, 2), 12);
}

void test_matrices_3x3_sub() {
    Matrix3x3 a(1, 2, 3, 4, 5, 6, 7, 8, 9);
    Matrix3x3 b(0, 1, 1, 0, 2, 2, 3, 3, 3);
    Matrix3x3 c;

    c = a - b;
    TEST_ASSERT_EQUAL(c.val(0, 0), 1);
    TEST_ASSERT_EQUAL(c.val(0, 1), 1);
    TEST_ASSERT_EQUAL(c.val(0, 2), 2);
    TEST_ASSERT_EQUAL(c.val(1, 0), 4);
    TEST_ASSERT_EQUAL(c.val(1, 1), 3);
    TEST_ASSERT_EQUAL(c.val(1, 2), 4);
    TEST_ASSERT_EQUAL(c.val(2, 0), 4);
    TEST_ASSERT_EQUAL(c.val(2, 1), 5);
    TEST_ASSERT_EQUAL(c.val(2, 2), 6);
}

void test_matrices_3x3_mul() {
    Matrix3x3 a(1, 2, 3, 4, 5, 6, 7, 8, 9);
    Matrix3x3 c;

    c = a * 2;
    TEST_ASSERT_EQUAL(c.val(0, 0), 2);
    TEST_ASSERT_EQUAL(c.val(0, 1), 4);
    TEST_ASSERT_EQUAL(c.val(0, 2), 6);
    TEST_ASSERT_EQUAL(c.val(1, 0), 8);
    TEST_ASSERT_EQUAL(c.val(1, 1), 10);
    TEST_ASSERT_EQUAL(c.val(1, 2), 12);
    TEST_ASSERT_EQUAL(c.val(2, 0), 14);
    TEST_ASSERT_EQUAL(c.val(2, 1), 16);
    TEST_ASSERT_EQUAL(c.val(2, 2), 18);
}

void test_matrices_3x3_div() {
    Matrix3x3 a(1, 2, 3, 4, 5, 6, 7, 8, 9);
    Matrix3x3 c;

    c = a / 2;
    TEST_ASSERT_EQUAL(c.val(0, 0), .5);
    TEST_ASSERT_EQUAL(c.val(0, 1), 1);
    TEST_ASSERT_EQUAL(c.val(0, 2), 1.5);
    TEST_ASSERT_EQUAL(c.val(1, 0), 2);
    TEST_ASSERT_EQUAL(c.val(1, 1), 2.5);
    TEST_ASSERT_EQUAL(c.val(1, 2), 3);
    TEST_ASSERT_EQUAL(c.val(2, 0), 3.5);
    TEST_ASSERT_EQUAL(c.val(2, 1), 4);
    TEST_ASSERT_EQUAL(c.val(2, 2), 4.5);
}

void test_matrices_3x3_mul_mat() {
    Matrix3x3 a(1, 2, 3, 4, 5, 6, 7, 8, 9);
    Matrix3x3 b(0, 1, 1, 0, 2, 2, 3, 3, 3);
    Matrix3x3 c;

    c = a * b;
    TEST_ASSERT_EQUAL(c.val(0, 0), 9);
    TEST_ASSERT_EQUAL(c.val(0, 1), 14);
    TEST_ASSERT_EQUAL(c.val(0, 2), 14);
    TEST_ASSERT_EQUAL(c.val(1, 0), 18);
    TEST_ASSERT_EQUAL(c.val(1, 1), 32);
    TEST_ASSERT_EQUAL(c.val(1, 2), 32);
    TEST_ASSERT_EQUAL(c.val(2, 0), 27);
    TEST_ASSERT_EQUAL(c.val(2, 1), 50);
    TEST_ASSERT_EQUAL(c.val(2, 2), 50);
}

void test_matrices_3x3_mul_vec() {
    Matrix3x3 a(1, 2, 3, 4, 5, 6, 7, 8, 9);
    Vector3D b(1, 2, 3);
    Vector3D c;

    c = a * b;
    TEST_ASSERT_EQUAL(c.x, 14);
    TEST_ASSERT_EQUAL(c.y, 32);
    TEST_ASSERT_EQUAL(c.z, 50);
}

void test_matrices_3x3_det() {
    Matrix3x3 a(1, 2, 3, 1, 5, 1, 7, 3, 2);
    TEST_ASSERT_EQUAL(a.det(), -79);
}

void test_matrices_3x3_inv() {
    Matrix3x3 a(1, 2, 3, 1, 5, 1, 7, 3, 2);
    Matrix3x3 b;

    TEST_ASSERT_TRUE(a.inverse(b));

    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(0, 0), -7/79.0);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(0, 1), -5/79.0);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(0, 2), 13/79.0);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(1, 0), -5/79.0);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(1, 1), 19/79.0);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(1, 2), -2/79.0);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(2, 0), 32/79.0);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(2, 1), -11/79.0);
    TEST_ASSERT_FLOAT_WITHIN(2e-6, b.val(2, 2), -3/79.0);
}
