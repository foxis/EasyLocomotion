#define QUATERNION_TESTS \
    _RUN_TEST(test_quaternion_constructor1); \
    _RUN_TEST(test_quaternion_constructor2); \
    _RUN_TEST(test_quaternion_constructor3); \
    _RUN_TEST(test_quaternion_constructor4); \
    _RUN_TEST(test_quaternion_assign); \
    _RUN_TEST(test_quaternion_add); \
    _RUN_TEST(test_quaternion_sub); \
    _RUN_TEST(test_quaternion_mul); \
    _RUN_TEST(test_quaternion_div); \
    _RUN_TEST(test_quaternion_add1); \
    _RUN_TEST(test_quaternion_sub1); \
    _RUN_TEST(test_quaternion_mul1); \
    _RUN_TEST(test_quaternion_div1);


void test_quaternion_constructor1() {
    Quaternion v(1);
    TEST_ASSERT_EQUAL(v.x, v.y);
    TEST_ASSERT_EQUAL(v.x, v.z);
    TEST_ASSERT_EQUAL(v.x, v.w);
    TEST_ASSERT_EQUAL(v.x, 1);
}
void test_quaternion_constructor2() {
    Quaternion v(1, 2, 3, 4);
    TEST_ASSERT_EQUAL(v.x, 1);
    TEST_ASSERT_EQUAL(v.y, 2);
    TEST_ASSERT_EQUAL(v.z, 3);
    TEST_ASSERT_EQUAL(v.w, 4);
    TEST_ASSERT_EQUAL(v.data()[0], 1);
    TEST_ASSERT_EQUAL(v.data()[1], 2);
    TEST_ASSERT_EQUAL(v.data()[2], 3);
    TEST_ASSERT_EQUAL(v.data()[3], 4);
}
void test_quaternion_constructor3() {
    real_t data[] = {2, 3, 4, 5};
    Quaternion v(data);
    TEST_ASSERT_EQUAL(v.x, 2);
    TEST_ASSERT_EQUAL(v.y, 3);
    TEST_ASSERT_EQUAL(v.z, 4);
    TEST_ASSERT_EQUAL(v.w, 5);
}
void test_quaternion_constructor4() {
    Quaternion data(4, 5, 1, 0);
    Quaternion v(data);
    TEST_ASSERT_EQUAL(v.x, 4);
    TEST_ASSERT_EQUAL(v.y, 5);
    TEST_ASSERT_EQUAL(v.z, 1);
    TEST_ASSERT_EQUAL(v.w, 0);
}

void test_quaternion_assign() {
    Quaternion data(4, 5, 1, 0);
    Quaternion v;

    v = data;
    TEST_ASSERT_EQUAL(v.x, 4);
    TEST_ASSERT_EQUAL(v.y, 5);
    TEST_ASSERT_EQUAL(v.z, 1);
    TEST_ASSERT_EQUAL(v.w, 0);
}

void test_quaternion_add() {
    Quaternion a(4, 5, 1, 1);
    Quaternion b(1, -1, 1, 2);

    TEST_ASSERT_EQUAL((a + b).x, 5);
    TEST_ASSERT_EQUAL((a + b).y, 4);
    TEST_ASSERT_EQUAL((a + b).z, 2);
    TEST_ASSERT_EQUAL((a + b).w, 3);
}

void test_quaternion_sub() {
    Quaternion a(4, 5, 1, 1);
    Quaternion b(1, -1, 1, 2);

    TEST_ASSERT_EQUAL((a - b).x, 3);
    TEST_ASSERT_EQUAL((a - b).y, 6);
    TEST_ASSERT_EQUAL((a - b).z, 0);
    TEST_ASSERT_EQUAL((a - b).w, -1);
}

void test_quaternion_mul() {
    Quaternion a(4, 5, 1, 3);

    TEST_ASSERT_EQUAL((a * 3.0).x, 12);
    TEST_ASSERT_EQUAL((a * 3.0).y, 15);
    TEST_ASSERT_EQUAL((a * 3.0).z, 3);
    TEST_ASSERT_EQUAL((a * 3.0).w, 9);
}

void test_quaternion_div() {
    Quaternion a(4, 5, 1, 3);

    TEST_ASSERT_EQUAL((a / 2).x, 2);
    TEST_ASSERT_EQUAL((a / 2).y, 2.5);
    TEST_ASSERT_EQUAL((a / 2).z, 0.5);
    TEST_ASSERT_EQUAL((a / 2).w, 1.5);
}


void test_quaternion_add1() {
    Quaternion a(4, 5, 1, 1);
    Quaternion b(1, -1, 1, 2);
    a += b;

    TEST_ASSERT_EQUAL(a.x, 5);
    TEST_ASSERT_EQUAL(a.y, 4);
    TEST_ASSERT_EQUAL(a.z, 2);
    TEST_ASSERT_EQUAL(a.w, 3);
}

void test_quaternion_sub1() {
    Quaternion a(4, 5, 1, 2);
    Quaternion b(1, -1, 2, 1);

    a -= b;
    TEST_ASSERT_EQUAL(a.x, 3);
    TEST_ASSERT_EQUAL(a.y, 6);
    TEST_ASSERT_EQUAL(a.z, -1);
    TEST_ASSERT_EQUAL(a.w, 1);
}

void test_quaternion_mul1() {
    Quaternion a(4, 5, 1, 3);

    a *= 3.0;
    TEST_ASSERT_EQUAL(a.x, 12);
    TEST_ASSERT_EQUAL(a.y, 15);
    TEST_ASSERT_EQUAL(a.z, 3);
    TEST_ASSERT_EQUAL(a.w, 9);
}

void test_quaternion_div1() {
    Quaternion a(4, 5, 1, 3);

    a /= 2;
    TEST_ASSERT_EQUAL(a.x, 2);
    TEST_ASSERT_EQUAL(a.y, 2.5);
    TEST_ASSERT_EQUAL(a.z, 0.5);
    TEST_ASSERT_EQUAL(a.w, 1.5);
}
