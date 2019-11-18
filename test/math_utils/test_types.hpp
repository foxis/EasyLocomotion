#define TYPES_TESTS \
    _RUN_TEST(test_sqrt); \
    _RUN_TEST(test_sign); \
    _RUN_TEST(test_sqr); \
    _RUN_TEST(test_swap); \
    _RUN_TEST(test_swap_rows); \
    _RUN_TEST(test_swap_cols); \
    _RUN_TEST(test_minmax_); \
    _RUN_TEST(test_set); \
    _RUN_TEST(test_get); \


void test_sqrt() 
{
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 1/3.0, FAST_ISQRT(9.0));
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 1/4.0, FAST_ISQRT(16.0));
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 1/2.5, FAST_ISQRT(2.5 * 2.5));
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 1/.145, FAST_ISQRT(.145 * .145));
}

void test_sign() 
{
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 1.4, SIGN(1.4, 1.0));
    TEST_ASSERT_FLOAT_WITHIN(1e-5, -1.4, SIGN(1.4, -1.0));
}

void test_sqr() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 1.4*1.4, SQR(-1.4));
}

void test_swap() {
    real_t a = 1, b = 2;
    SWAP(&a, &b);

    TEST_ASSERT_FLOAT_WITHIN(1e-5, 2, a);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 1, b);
}

void test_swap_rows() {
    real_t a[] = {
        1, 2, 
        3, 4,
    };
    SWAP_ROWS<real_t, 2>(a, (real_t*)a + 2);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 3, a[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 4, a[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 1, a[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 2, a[3]);
}

void test_swap_cols() {
    real_t a[] = {
        1, 2, 
        3, 4,
    };
    SWAP_COLS<real_t, 2>(a, (real_t*)a + 1);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 2, a[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 1, a[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 4, a[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 3, a[3]);
}

void test_minmax_() {
    const real_t a[] = {
        1, 2, 
        3, 4,
    };
    real_t mn, mx;

    mn = MIN_ROW<real_t, 2>(a);
    mx = MAX_ROW<real_t, 2>(a);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 1, mn);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 2, mx);

    mn = MIN_COL<real_t, 2>((real_t*)a + 1);
    mx = MAX_COL<real_t, 2>((real_t*)a + 1);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 2, mn);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 4, mx);

    TEST_ASSERT_FLOAT_WITHIN(1e-5, 1, MIN(1, 15));
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 1, MIN(15, 1));
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 15, MAX(1, 15));
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 15, MAX(15, 1));
}

void test_set() {
    real_t a[] = {
        1, 2, 
        3, 4,
    };
    const real_t b[] = {
        1, 2, 
        3, 4,
    };

    SET_ROW<real_t, 2>(a, 5);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 5, a[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 5, a[1]);
    SET_COL<real_t, 2, 1>(a, 7);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 7, a[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 3, a[2]);
    SET_ROW<real_t, 2>(a, b);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 1, a[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 2, a[1]);
    SET_COL<real_t, 2>(a, (real_t*)b + 2);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 3, a[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 4, a[2]);
}

void test_get() {
    const real_t a[] = {
        1, 2, 
        3, 4,
    };
    real_t b[] = {
        0, 0, 
    };

    GET_ROW<real_t, 2>(b, a);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 1, b[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 2, b[1]);
    GET_COL<real_t, 2>(b, (real_t*)a + 1);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 2, b[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 4, b[1]);
}
