#define MATRICES_TESTS \
    _RUN_TEST(test_matrix_lu); \
    _RUN_TEST(test_matrix_svd); \
    _RUN_TEST(test_matrix_svd1); \
    _RUN_TEST(test_matrix_pinv); \
    _RUN_TEST(test_matrix_pinv1); \


void test_matrix_lu() {
    real_t data[] = {
        2, -1, -2,
        -4, 6, 3,
        -4, -2, 8
    };

    real_t L[] = {
        1, 0, 0,
        -2, 1, 0,
        -2, -1, 1
    };
    real_t U[] = {
        2, -1, -2,
        0, 4, -1,
        0, 0, 3,
    };
    Matrix3x3 m(data);
    Matrix3x3 ml(0.0), mu(0.0);

    print_duration(m.lu(ml, mu));

    TEST_ASSERT_FLOAT_WITHIN(1e-3, L[0], ml.data()[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, L[1], ml.data()[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, L[2], ml.data()[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, L[3], ml.data()[3]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, L[4], ml.data()[4]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, L[5], ml.data()[5]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, L[6], ml.data()[6]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, L[7], ml.data()[7]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, L[8], ml.data()[8]);

    TEST_ASSERT_FLOAT_WITHIN(1e-3, U[0], mu.data()[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, U[1], mu.data()[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, U[2], mu.data()[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, U[3], mu.data()[3]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, U[4], mu.data()[4]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, U[5], mu.data()[5]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, U[6], mu.data()[6]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, U[7], mu.data()[7]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, U[8], mu.data()[8]);
}

void test_matrix_svd() {
    real_t data[] = {
        2,	2,	2,	2,
        1,	-1,	1,	-1,
        -1,	1,	-1,	1,
    };

    MatrixStatic<3, 4> m(data);
    MatrixStatic<3, 3> u;
    MatrixStatic<4, 4> v;
    VectorStatic<4> w;
    MatrixStatic<4, 3> tmpx;
    VectorStatic<4> tmp;

    unsigned long now = micros();
    bool result = m.svd(u, w, v, tmp, tmpx);
    print_val((micros() - now), "computation time us");

    TEST_ASSERT_TRUE(result);

    print_matrix(u, "U");
    print_vector(w,"W");
    print_matrix(v, "V");
    {
        MatrixStatic<3, 4> _W(0.0);
        MatrixStatic<3, 4> uw(0.0);
        MatrixStatic<3, 4> uwv(0.0);
        _W.set_diagonal(w);
        u.mul_mat(_W, uw);
        v.transpose();
        uw.mul_mat(v, uwv);
        print_matrix(uwv, "uwv");

        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[0], uwv.data()[0]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[1], uwv.data()[1]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[2], uwv.data()[2]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[3], uwv.data()[3]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[4], uwv.data()[4]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[5], uwv.data()[5]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[6], uwv.data()[6]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[7], uwv.data()[7]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[8], uwv.data()[8]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[9], uwv.data()[9]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[10], uwv.data()[10]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[11], uwv.data()[11]);
    }
}

void test_matrix_svd1() {
    real_t data[] = {
        2,	2,	2,	2,
        1,	-1,	1,	-1,
        -1,	1,	-1,	1,
        -2,	3,	-3,	6,
    };

    MatrixStatic<4, 4> m(data);
    MatrixStatic<4, 4> u;
    MatrixStatic<4, 4> v;
    VectorStatic<4> w;
    MatrixStatic<4, 4> tmpx;
    VectorStatic<4> tmp;

    unsigned long now = micros();
    bool result = m.svd(u, w, v, tmp, tmpx);
    print_val((micros() - now), "computation time us");

    TEST_ASSERT_TRUE(result);

    print_matrix(u, "U");
    print_vector(w, "W");
    print_matrix(v, "V");
    {
        MatrixStatic<4, 4> _W(0.0);
        MatrixStatic<4, 4> uw(0.0);
        MatrixStatic<4, 4> uwv(0.0);
        _W.set_diagonal(w);
        u.mul_mat(_W, uw);
        v.transpose();
        uw.mul_mat(v, uwv);
        print_matrix(uwv, "uwv");

        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[0], uwv.data()[0]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[1], uwv.data()[1]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[2], uwv.data()[2]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[3], uwv.data()[3]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[4], uwv.data()[4]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[5], uwv.data()[5]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[6], uwv.data()[6]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[7], uwv.data()[7]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[8], uwv.data()[8]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[9], uwv.data()[9]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[10], uwv.data()[10]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[11], uwv.data()[11]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[12], uwv.data()[12]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[13], uwv.data()[13]);
        TEST_ASSERT_FLOAT_WITHIN(1e-4, data[14], uwv.data()[14]);
    }
}

void test_matrix_pinv() {
    real_t data[] = {
        2,	2,	2,	2,
        1,	-1,	1,	-1,
        -1,	1,	-1,	1,
    };

    real_t A[] = {
        0.125,	0.125,	-0.125,	
        0.125,	-0.125,	0.125,	
        0.125,	0.125,	-0.125,	
        0.125,	-0.125,	0.125,	
    };

    MatrixStatic<3, 4> m(data);
    MatrixStatic<4, 3> a;

    unsigned long now = micros();
    bool result = m.pinv(a);
    print_val((micros() - now), "computation time us");

    TEST_ASSERT_TRUE(result);

    print_matrix(a, "Pseudo Inverse");

    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[0], a.data()[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[1], a.data()[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[2], a.data()[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[3], a.data()[3]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[4], a.data()[4]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[5], a.data()[5]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[6], a.data()[6]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[7], a.data()[7]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[8], a.data()[8]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[9], a.data()[9]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[10], a.data()[10]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[11], a.data()[11]);
}

void test_matrix_pinv1() {
    real_t data[] = {
        2, 1, -1, 
        2, -1, 1, 
        2, 1, -1, 
        2, -1, 1
    };

    real_t A[] = {
        0.125, 0.125, 0.125, 0.125,
        0.125, -0.125, 0.125, -0.125,
        -0.125, 0.125, -0.125, 0.125,
    };

    MatrixStatic<4, 3> m(data);
    MatrixStatic<3, 4> a;

    unsigned long now = micros();
    bool result = m.pinv(a);
    print_val((micros() - now), "computation time us");

    TEST_ASSERT_TRUE(result);

    print_matrix(a, "Pseudo Inverse");

    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[0], a.data()[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[1], a.data()[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[2], a.data()[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[3], a.data()[3]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[4], a.data()[4]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[5], a.data()[5]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[6], a.data()[6]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[7], a.data()[7]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[8], a.data()[8]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[9], a.data()[9]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[10], a.data()[10]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, A[11], a.data()[11]);
}
