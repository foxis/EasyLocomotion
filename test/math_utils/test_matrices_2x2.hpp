#define MATRICES_2X2_TESTS \
    _RUN_TEST(test_matrices_2x2_constructor); \
    _RUN_TEST(test_matrices_2x2_transpose); \
    _RUN_TEST(test_matrices_2x2_add); \
    _RUN_TEST(test_matrices_2x2_sub); \
    _RUN_TEST(test_matrices_2x2_mul); \
    _RUN_TEST(test_matrices_2x2_mul_mat); \
    _RUN_TEST(test_matrices_2x2_mul_vec); \
    _RUN_TEST(test_matrices_2x2_div); \
    _RUN_TEST(test_matrices_2x2_det); \
    _RUN_TEST(test_matrices_2x2_inv);


void test_matrices_2x2_constructor() {
    real_t data[] = {0, 1, 1, 2};
    Matrix2x2 a;
    Matrix2x2 b(1);
    Matrix2x2 c(1, 0, 2, 1);
    Matrix2x2 d(data);
    Matrix2x2 e(c);
    Matrix2x2 f;

    a.eye();
    f = d;

    TEST_ASSERT_EQUAL(a.val(0, 0), 1);
    TEST_ASSERT_EQUAL(a.val(0, 1), 0);
    TEST_ASSERT_EQUAL(a.val(1, 0), 0);
    TEST_ASSERT_EQUAL(a.val(1, 1), 1);

    TEST_ASSERT_EQUAL(b.val(0, 0), 1);
    TEST_ASSERT_EQUAL(b.val(0, 1), 1);
    TEST_ASSERT_EQUAL(b.val(1, 0), 1);
    TEST_ASSERT_EQUAL(b.val(1, 1), 1);

    TEST_ASSERT_EQUAL(c.val(0, 0), 1);
    TEST_ASSERT_EQUAL(c.val(0, 1), 0);
    TEST_ASSERT_EQUAL(c.val(1, 0), 2);
    TEST_ASSERT_EQUAL(c.val(1, 1), 1);

    TEST_ASSERT_EQUAL(d.val(0, 0), 0);
    TEST_ASSERT_EQUAL(d.val(0, 1), 1);
    TEST_ASSERT_EQUAL(d.val(1, 0), 1);
    TEST_ASSERT_EQUAL(d.val(1, 1), 2);

    TEST_ASSERT_EQUAL(e.val(0, 0), 1);
    TEST_ASSERT_EQUAL(e.val(0, 1), 0);
    TEST_ASSERT_EQUAL(e.val(1, 0), 2);
    TEST_ASSERT_EQUAL(e.val(1, 1), 1);

    TEST_ASSERT_EQUAL(f.val(0, 0), 0);
    TEST_ASSERT_EQUAL(f.val(0, 1), 1);
    TEST_ASSERT_EQUAL(f.val(1, 0), 1);
    TEST_ASSERT_EQUAL(f.val(1, 1), 2);
}

void test_matrices_2x2_transpose() {
    Matrix2x2 a(1, 2, 3, 4);
    Matrix2x2 b;

    a.transpose(b);
    TEST_ASSERT_EQUAL(b.val(0, 0), 1);
    TEST_ASSERT_EQUAL(b.val(0, 1), 3);
    TEST_ASSERT_EQUAL(b.val(1, 0), 2);
    TEST_ASSERT_EQUAL(b.val(1, 1), 4);
}

void test_matrices_2x2_add() {
    Matrix2x2 a(1, 2, 3, 4);
    Matrix2x2 b(0, 1, 1, 0);
    Matrix2x2 c;

    c = a + b;

    TEST_ASSERT_EQUAL(c.val(0, 0), 1);
    TEST_ASSERT_EQUAL(c.val(0, 1), 3);
    TEST_ASSERT_EQUAL(c.val(1, 0), 4);
    TEST_ASSERT_EQUAL(c.val(1, 1), 4);
}

void test_matrices_2x2_sub() {
    Matrix2x2 a(1, 2, 3, 4);
    Matrix2x2 b(0, 1, 1, 0);
    Matrix2x2 c;

    c = a - b;
    TEST_ASSERT_EQUAL(c.val(0, 0), 1);
    TEST_ASSERT_EQUAL(c.val(0, 1), 1);
    TEST_ASSERT_EQUAL(c.val(1, 0), 2);
    TEST_ASSERT_EQUAL(c.val(1, 1), 4);
}

void test_matrices_2x2_mul() {
    Matrix2x2 a(1, 2, 3, 4);
    Matrix2x2 c;

    c = a * 7;
    TEST_ASSERT_EQUAL(c.val(0, 0), 7);
    TEST_ASSERT_EQUAL(c.val(0, 1), 14);
    TEST_ASSERT_EQUAL(c.val(1, 0), 21);
    TEST_ASSERT_EQUAL(c.val(1, 1), 28);
}

void test_matrices_2x2_div() {
    Matrix2x2 a(1, 2, 3, 4);
    Matrix2x2 c;

    c = a / 2;
    TEST_ASSERT_EQUAL(c.val(0, 0), .5);
    TEST_ASSERT_EQUAL(c.val(0, 1), 1);
    TEST_ASSERT_EQUAL(c.val(1, 0), 1.5);
    TEST_ASSERT_EQUAL(c.val(1, 1), 2);
}

void test_matrices_2x2_mul_mat() {
    Matrix2x2 a(1, 2, 3, 4);
    Matrix2x2 b(0, 1, 1, 0);
    Matrix2x2 c;

    c = a * b;
    TEST_ASSERT_EQUAL(c.val(0, 0), 2);
    TEST_ASSERT_EQUAL(c.val(0, 1), 1);
    TEST_ASSERT_EQUAL(c.val(1, 0), 4);
    TEST_ASSERT_EQUAL(c.val(1, 1), 3);
}

void test_matrices_2x2_mul_vec() {
    Matrix2x2 a(1, 2, 3, 4);
    Vector2D b(0, 1);
    Vector2D c;

    c = a * b;
    TEST_ASSERT_EQUAL(c.x, 2);
    TEST_ASSERT_EQUAL(c.y, 4);
}

void test_matrices_2x2_det() {
    Matrix2x2 a(1, 2, 3, 4);
    TEST_ASSERT_EQUAL(a.det(), 1 * 4 - 2 * 3);
}

void test_matrices_2x2_inv() {
    Matrix2x2 a(1, 2, 3, 4);
    Matrix2x2 b;

    a.inverse(b);

    TEST_ASSERT_EQUAL(b.val(0, 0), -2);
    TEST_ASSERT_EQUAL(b.val(0, 1), 1);
    TEST_ASSERT_EQUAL(b.val(1, 0), 1.5);
    TEST_ASSERT_EQUAL(b.val(1, 1), -.5);
}
