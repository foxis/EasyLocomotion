#define VECTORS4D_TESTS \
    _RUN_TEST(test_vectors4D_constructor1); \
    _RUN_TEST(test_vectors4D_constructor2); \
    _RUN_TEST(test_vectors4D_constructor3); \
    _RUN_TEST(test_vectors4D_constructor4); \
    _RUN_TEST(test_vectors4D_assign); \
    _RUN_TEST(test_vectors4D_add); \
    _RUN_TEST(test_vectors4D_sub); \
    _RUN_TEST(test_vectors4D_mul); \
    _RUN_TEST(test_vectors4D_div); \
    _RUN_TEST(test_vectors4D_add1); \
    _RUN_TEST(test_vectors4D_sub1); \
    _RUN_TEST(test_vectors4D_mul1); \
    _RUN_TEST(test_vectors4D_div1);


void test_vectors4D_constructor1() {
    Vector4D v(1);
    TEST_ASSERT_EQUAL(v.x, v.y);
    TEST_ASSERT_EQUAL(v.x, v.z);
    TEST_ASSERT_EQUAL(v.x, v.w);
    TEST_ASSERT_EQUAL(v.x, 1);
}
void test_vectors4D_constructor2() {
    Vector4D v(1, 2, 3, 4);
    TEST_ASSERT_EQUAL(v.x, 1);
    TEST_ASSERT_EQUAL(v.y, 2);
    TEST_ASSERT_EQUAL(v.z, 3);
    TEST_ASSERT_EQUAL(v.w, 4);
    TEST_ASSERT_EQUAL(v.data()[0], 1);
    TEST_ASSERT_EQUAL(v.data()[1], 2);
    TEST_ASSERT_EQUAL(v.data()[2], 3);
    TEST_ASSERT_EQUAL(v.data()[3], 4);
}
void test_vectors4D_constructor3() {
    real_t data[] = {2, 3, 4, 5};
    Vector4D v(data);
    TEST_ASSERT_EQUAL(v.x, 2);
    TEST_ASSERT_EQUAL(v.y, 3);
    TEST_ASSERT_EQUAL(v.z, 4);
    TEST_ASSERT_EQUAL(v.w, 5);
}
void test_vectors4D_constructor4() {
    Vector4D data(4, 5, 1, 0);
    Vector4D v(data);
    TEST_ASSERT_EQUAL(v.x, 4);
    TEST_ASSERT_EQUAL(v.y, 5);
    TEST_ASSERT_EQUAL(v.z, 1);
    TEST_ASSERT_EQUAL(v.w, 0);
}

void test_vectors4D_assign() {
    Vector4D data(4, 5, 1, 0);
    Vector4D v;

    v = data;
    TEST_ASSERT_EQUAL(v.x, 4);
    TEST_ASSERT_EQUAL(v.y, 5);
    TEST_ASSERT_EQUAL(v.z, 1);
    TEST_ASSERT_EQUAL(v.w, 0);
}

void test_vectors4D_add() {
    Vector4D a(4, 5, 1, 1);
    Vector4D b(1, -1, 1, 2);

    TEST_ASSERT_EQUAL((a + b).x, 5);
    TEST_ASSERT_EQUAL((a + b).y, 4);
    TEST_ASSERT_EQUAL((a + b).z, 2);
    TEST_ASSERT_EQUAL((a + b).w, 3);
}

void test_vectors4D_sub() {
    Vector4D a(4, 5, 1, 1);
    Vector4D b(1, -1, 1, 2);

    TEST_ASSERT_EQUAL((a - b).x, 3);
    TEST_ASSERT_EQUAL((a - b).y, 6);
    TEST_ASSERT_EQUAL((a - b).z, 0);
    TEST_ASSERT_EQUAL((a - b).w, -1);
}

void test_vectors4D_mul() {
    Vector4D a(4, 5, 1, 3);

    TEST_ASSERT_EQUAL((a * 3).x, 12);
    TEST_ASSERT_EQUAL((a * 3).y, 15);
    TEST_ASSERT_EQUAL((a * 3).z, 3);
    TEST_ASSERT_EQUAL((a * 3).w, 9);
}

void test_vectors4D_div() {
    Vector4D a(4, 5, 1, 3);

    TEST_ASSERT_EQUAL((a / 2).x, 2);
    TEST_ASSERT_EQUAL((a / 2).y, 2.5);
    TEST_ASSERT_EQUAL((a / 2).z, 0.5);
    TEST_ASSERT_EQUAL((a / 2).w, 1.5);
}


void test_vectors4D_add1() {
    Vector4D a(4, 5, 1, 1);
    Vector4D b(1, -1, 1, 2);
    a += b;

    TEST_ASSERT_EQUAL(a.x, 5);
    TEST_ASSERT_EQUAL(a.y, 4);
    TEST_ASSERT_EQUAL(a.z, 2);
    TEST_ASSERT_EQUAL(a.w, 3);
}

void test_vectors4D_sub1() {
    Vector4D a(4, 5, 1, 2);
    Vector4D b(1, -1, 2, 1);

    a -= b;
    TEST_ASSERT_EQUAL(a.x, 3);
    TEST_ASSERT_EQUAL(a.y, 6);
    TEST_ASSERT_EQUAL(a.z, -1);
    TEST_ASSERT_EQUAL(a.w, 1);
}

void test_vectors4D_mul1() {
    Vector4D a(4, 5, 1, 3);

    a *= 3;
    TEST_ASSERT_EQUAL(a.x, 12);
    TEST_ASSERT_EQUAL(a.y, 15);
    TEST_ASSERT_EQUAL(a.z, 3);
    TEST_ASSERT_EQUAL(a.w, 9);
}

void test_vectors4D_div1() {
    Vector4D a(4, 5, 1, 3);

    a /= 2;
    TEST_ASSERT_EQUAL(a.x, 2);
    TEST_ASSERT_EQUAL(a.y, 2.5);
    TEST_ASSERT_EQUAL(a.z, 0.5);
    TEST_ASSERT_EQUAL(a.w, 1.5);
}
