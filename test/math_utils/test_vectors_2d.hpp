#define VECTORS2D_TESTS \
    _RUN_TEST(test_vectors2D_constructor1); \
    _RUN_TEST(test_vectors2D_constructor2); \
    _RUN_TEST(test_vectors2D_constructor3); \
    _RUN_TEST(test_vectors2D_constructor4); \
    _RUN_TEST(test_vectors2D_assign); \
    _RUN_TEST(test_vectors2D_assign_data); \
    _RUN_TEST(test_vectors2D_assign_value); \
    _RUN_TEST(test_vectors2D_add); \
    _RUN_TEST(test_vectors2D_sub); \
    _RUN_TEST(test_vectors2D_mul); \
    _RUN_TEST(test_vectors2D_div); \
    _RUN_TEST(test_vectors2D_add1); \
    _RUN_TEST(test_vectors2D_sub1); \
    _RUN_TEST(test_vectors2D_mul1); \
    _RUN_TEST(test_vectors2D_div1); \
    _RUN_TEST(test_vectors2D_dot); \
    _RUN_TEST(test_vectors2D_linear_interpolate); \
    _RUN_TEST(test_vectors2D_reflection);


void test_vectors2D_constructor1() {
    Vector2D v(1);
    TEST_ASSERT_EQUAL(v.x, v.y);
    TEST_ASSERT_EQUAL(v.x, 1);
}
void test_vectors2D_constructor2() {
    Vector2D v(11, 2);
    TEST_ASSERT_EQUAL(v.x, 11);
    TEST_ASSERT_EQUAL(v.y, 2);
    TEST_ASSERT_EQUAL(v.data()[0], 11);
    TEST_ASSERT_EQUAL(v.data()[1], 2);
    v.data()[0] = 7;
    v.data()[1] = 13;
    TEST_ASSERT_EQUAL(v.x, 7);
    TEST_ASSERT_EQUAL(v.y, 13);
}
void test_vectors2D_constructor3() {
    real_t data[] = {2, 3, 4};
    Vector2D v(data);
    TEST_ASSERT_EQUAL(v.x, 2);
    TEST_ASSERT_EQUAL(v.y, 3);
}
void test_vectors2D_constructor4() {
    Vector2D data(4, 5);
    Vector2D v(data);
    TEST_ASSERT_EQUAL(v.x, 4);
    TEST_ASSERT_EQUAL(v.y, 5);
}

void test_vectors2D_assign() {
    Vector2D data(4, 5);
    Vector2D v;

    v = data;
    TEST_ASSERT_EQUAL(v.x, 4);
    TEST_ASSERT_EQUAL(v.y, 5);
}

void test_vectors2D_assign_data() {
    Vector2D v;

    v.data()[0] = 1;
    v.data()[1] = 2;
    TEST_ASSERT_EQUAL(v.x, 1);
    TEST_ASSERT_EQUAL(v.y, 2);
}

void test_vectors2D_assign_value() {
    Vector2D v;

    v.x = 1;
    v.y = 2;
    TEST_ASSERT_EQUAL(v.data()[0], 1);
    TEST_ASSERT_EQUAL(v.data()[1], 2);
}

void test_vectors2D_add() {
    Vector2D a(4, 5);
    Vector2D b(1, -1);

    TEST_ASSERT_EQUAL((a + b).x, 5);
    TEST_ASSERT_EQUAL((a + b).y, 4);
}

void test_vectors2D_sub() {
    Vector2D a(4, 5);
    Vector2D b(1, -1);

    TEST_ASSERT_EQUAL((a - b).x, 3);
    TEST_ASSERT_EQUAL((a - b).y, 6);
}

void test_vectors2D_mul() {
    Vector2D a(4, 5);

    TEST_ASSERT_EQUAL((a * 3).x, 12);
    TEST_ASSERT_EQUAL((a * 3).y, 15);
}

void test_vectors2D_div() {
    Vector2D a(4, 5);

    TEST_ASSERT_EQUAL((a / 2).x, 2);
    TEST_ASSERT_EQUAL((a / 2).y, 2.5);
}


void test_vectors2D_add1() {
    Vector2D a(4, 5);
    Vector2D b(1, -1);
    a +=  b;

    TEST_ASSERT_EQUAL(a.x, 5);
    TEST_ASSERT_EQUAL(a.y, 4);
}

void test_vectors2D_sub1() {
    Vector2D a(4, 5);
    Vector2D b(1, -1);

    a -= b;
    TEST_ASSERT_EQUAL(a.x, 3);
    TEST_ASSERT_EQUAL(a.y, 6);
}

void test_vectors2D_mul1() {
    Vector2D a(4, 5);

    a *= 3;
    TEST_ASSERT_EQUAL(a.x, 12);
    TEST_ASSERT_EQUAL(a.y, 15);
}

void test_vectors2D_div1() {
    Vector2D a(4, 5);

    a /= 2;
    TEST_ASSERT_EQUAL(a.x, 2);
    TEST_ASSERT_EQUAL(a.y, 2.5);
}


void test_vectors2D_dot() {
    Vector2D a(4, 5);
    Vector2D b(3, 2);

    TEST_ASSERT_EQUAL(a.dot(b), 4 * 3 + 5 * 2);
}

void test_vectors2D_linear_interpolate() {
    Vector2D a(-4, 5);
    Vector2D b(4, 5);
    Vector2D c;

    Vector2D::linear_interpolate(a, b, c, 1/8.0);

    TEST_ASSERT_EQUAL(c.x, -3);
    TEST_ASSERT_EQUAL(c.y, 5);
}

void test_vectors2D_reflection() {
    Vector2D a(-4, 5);
    Vector2D b(0, 1);
    Vector2D c;

    Vector2D::reflection(b, a, c);

    TEST_ASSERT_EQUAL(c.x, 4);
    TEST_ASSERT_EQUAL(c.y, 5);
}

void test_vectors2D_magnitude() {
    Vector2D a(-2, 5);

    TEST_ASSERT_EQUAL((int)a.magnitude(), 5);
}

void test_vectors2D_normal() {
    Vector2D a(-2, 5);
    Vector2D n;
    a.normal(n);
    TEST_ASSERT_EQUAL((int)(n.x * 100), 37);
    TEST_ASSERT_EQUAL((int)(n.y * 100), 92);
    a.normal();
    TEST_ASSERT_EQUAL((int)(a.x * 100), 37);
    TEST_ASSERT_EQUAL((int)(a.y * 100), 92);
}
