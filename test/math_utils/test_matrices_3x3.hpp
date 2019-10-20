#define MATRICES_3X3_TESTS \
    _RUN_TEST(test_matrices_3x3_constructor);


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
