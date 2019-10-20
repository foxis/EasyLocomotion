#define VECTORS_TESTS \
    _RUN_TEST(test_vectors_convolve); \
    _RUN_TEST(test_vectors_convolve1);

void test_vectors_convolve() {
    real_t data[] = {1, 2, 3, 4, 5, 6};
    real_t kernel[] = {1, -1, 1};
    _DataContainerStatic<real_t, 6> cont1;
    _DataContainerStatic<real_t, 3> cont2;
    _DataContainerStatic<real_t, 4> cont3;
    
    _Vector<real_t, 6> d(cont1, data);
    _Vector<real_t, 3> k(cont2, kernel);
    _Vector<real_t, 4> r(cont3);

    d.convolve<3, 4>(k, r);

    TEST_ASSERT_EQUAL(r.data()[0], 2);
    TEST_ASSERT_EQUAL(r.data()[1], 3);
    TEST_ASSERT_EQUAL(r.data()[2], 4);
    TEST_ASSERT_EQUAL(r.data()[3], 5);
}

void test_vectors_convolve1() {
    real_t data[] = {1, 2, 3, 4, 5, 6};
    real_t kernel[] = {1, -1, 1};
    _DataContainerStatic<real_t, 6> cont1;
    _DataContainerStatic<real_t, 3> cont2;
    _DataContainerStatic<real_t, 6> cont3;
    
    _Vector<real_t, 6> d(cont1, data);
    _Vector<real_t, 3> k(cont2, kernel);
    _Vector<real_t, 6> r(cont3);

    d.convolve<3, 6>(k, r);

    TEST_ASSERT_EQUAL(r.data()[0], 1);
    TEST_ASSERT_EQUAL(r.data()[1], 2);
    TEST_ASSERT_EQUAL(r.data()[2], 3);
    TEST_ASSERT_EQUAL(r.data()[3], 4);
    TEST_ASSERT_EQUAL(r.data()[4], 5);
    TEST_ASSERT_EQUAL(r.data()[5], -1);
}
