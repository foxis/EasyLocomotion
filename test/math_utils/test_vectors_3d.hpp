#define VECTORS3D_TESTS \
    _RUN_TEST(test_vectors3D_constructor1); \
    _RUN_TEST(test_vectors3D_constructor2); \
    _RUN_TEST(test_vectors3D_constructor3); \
    _RUN_TEST(test_vectors3D_constructor4); \
    _RUN_TEST(test_vectors3D_assign); \
    _RUN_TEST(test_vectors3D_add); \
    _RUN_TEST(test_vectors3D_sub); \
    _RUN_TEST(test_vectors3D_mul); \
    _RUN_TEST(test_vectors3D_div); \
    _RUN_TEST(test_vectors3D_add1); \
    _RUN_TEST(test_vectors3D_sub1); \
    _RUN_TEST(test_vectors3D_mul1); \
    _RUN_TEST(test_vectors3D_div1);


void test_vectors3D_constructor1() {
    Vector3D v(1);
    TEST_ASSERT_EQUAL(v.x, v.y);
    TEST_ASSERT_EQUAL(v.x, v.z);
    TEST_ASSERT_EQUAL(v.x, 1);
}
void test_vectors3D_constructor2() {
    Vector3D v(1, 2, 3);
    TEST_ASSERT_EQUAL(v.x, 1);
    TEST_ASSERT_EQUAL(v.y, 2);
    TEST_ASSERT_EQUAL(v.z, 3);
    TEST_ASSERT_EQUAL(v.data()[0], 1);
    TEST_ASSERT_EQUAL(v.data()[1], 2);
    TEST_ASSERT_EQUAL(v.data()[2], 3);
}
void test_vectors3D_constructor3() {
    real_t data[] = {2, 3, 4};
    Vector3D v(data);
    TEST_ASSERT_EQUAL(v.x, 2);
    TEST_ASSERT_EQUAL(v.y, 3);
    TEST_ASSERT_EQUAL(v.z, 4);
}
void test_vectors3D_constructor4() {
    Vector3D data(4, 5, 1);
    Vector3D v(data);
    TEST_ASSERT_EQUAL(v.x, 4);
    TEST_ASSERT_EQUAL(v.y, 5);
    TEST_ASSERT_EQUAL(v.z, 1);
}

void test_vectors3D_assign() {
    Vector3D data(4, 5, 1);
    Vector3D v;

    v = data;
    TEST_ASSERT_EQUAL(v.x, 4);
    TEST_ASSERT_EQUAL(v.y, 5);
    TEST_ASSERT_EQUAL(v.z, 1);
}

void test_vectors3D_add() {
    Vector3D a(4, 5, 1);
    Vector3D b(1, -1, 1);

    TEST_ASSERT_EQUAL((a + b).x, 5);
    TEST_ASSERT_EQUAL((a + b).y, 4);
    TEST_ASSERT_EQUAL((a + b).z, 2);
}

void test_vectors3D_sub() {
    Vector3D a(4, 5, 1);
    Vector3D b(1, -1, 1);

    TEST_ASSERT_EQUAL((a - b).x, 3);
    TEST_ASSERT_EQUAL((a - b).y, 6);
    TEST_ASSERT_EQUAL((a - b).z, 0);
}

void test_vectors3D_mul() {
    Vector3D a(4, 5, 1);

    TEST_ASSERT_EQUAL((a * 3).x, 12);
    TEST_ASSERT_EQUAL((a * 3).y, 15);
    TEST_ASSERT_EQUAL((a * 3).z, 3);
}

void test_vectors3D_div() {
    Vector3D a(4, 5, 1);

    TEST_ASSERT_EQUAL((a / 2).x, 2);
    TEST_ASSERT_EQUAL((a / 2).y, 2.5);
    TEST_ASSERT_EQUAL((a / 2).z, 0.5);
}


void test_vectors3D_add1() {
    Vector3D a(4, 5, 1);
    Vector3D b(1, -1, 1);
    a += b;

    TEST_ASSERT_EQUAL(a.x, 5);
    TEST_ASSERT_EQUAL(a.y, 4);
    TEST_ASSERT_EQUAL(a.z, 2);
}

void test_vectors3D_sub1() {
    Vector3D a(4, 5, 1);
    Vector3D b(1, -1, 2);

    a -= b;
    TEST_ASSERT_EQUAL(a.x, 3);
    TEST_ASSERT_EQUAL(a.y, 6);
    TEST_ASSERT_EQUAL(a.z, -1);
}

void test_vectors3D_mul1() {
    Vector3D a(4, 5, 1);

    a *= 3;
    TEST_ASSERT_EQUAL(a.x, 12);
    TEST_ASSERT_EQUAL(a.y, 15);
    TEST_ASSERT_EQUAL(a.z, 3);
}

void test_vectors3D_div1() {
    Vector3D a(4, 5, 1);

    a /= 2;
    TEST_ASSERT_EQUAL(a.x, 2);
    TEST_ASSERT_EQUAL(a.y, 2.5);
    TEST_ASSERT_EQUAL(a.z, 0.5);
}
