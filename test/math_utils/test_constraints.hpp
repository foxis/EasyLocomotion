#include "math_utils/constraints.h"

#define CONSTRAINTS_TESTS \
    _RUN_TEST(test_constraints_constructors); \
    _RUN_TEST(test_constraints_constructors1); \
    _RUN_TEST(test_constraints_constructors2); \
    _RUN_TEST(test_constraints_constructors3); \
    _RUN_TEST(test_constraints_in_s); \
    _RUN_TEST(test_constraints_in_a); \
    _RUN_TEST(test_constraints_in_v); \
    _RUN_TEST(test_constraints_length); \
    _RUN_TEST(test_constraints_area); \
    _RUN_TEST(test_constraints_volume); \
    _RUN_TEST(test_constraints_limit_s); \
    _RUN_TEST(test_constraints_limit_a); \
    _RUN_TEST(test_constraints_limit_v); \
    _RUN_TEST(test_constraints_middle_s); \
    _RUN_TEST(test_constraints_middle_a); \
    _RUN_TEST(test_constraints_middle_v);

void test_constraints_constructors() {
    ConstraintSegment segment;
    ConstraintArea area;
    ConstraintVolume volume;

    TEST_ASSERT_EQUAL(segment.min, 0);
    TEST_ASSERT_EQUAL(segment.max, 0);

    TEST_ASSERT_EQUAL(area.min.x, 0);
    TEST_ASSERT_EQUAL(area.min.y, 0);
    TEST_ASSERT_EQUAL(area.max.x, 0);
    TEST_ASSERT_EQUAL(area.max.y, 0);

    TEST_ASSERT_EQUAL(volume.min.x, 0);
    TEST_ASSERT_EQUAL(volume.min.y, 0);
    TEST_ASSERT_EQUAL(volume.min.z, 0);
    TEST_ASSERT_EQUAL(volume.max.x, 0);
    TEST_ASSERT_EQUAL(volume.max.y, 0);
    TEST_ASSERT_EQUAL(volume.max.z, 0);
}

void test_constraints_constructors1() {
    ConstraintSegment segment1(-1, 1);
    ConstraintArea area1(Vector2D(-1, -2), Vector2D(1, 2));
    ConstraintVolume volume1(Vector3D(-1, -2, -3), Vector3D(1, 2, 3));

    TEST_ASSERT_EQUAL(segment1.min, -1);
    TEST_ASSERT_EQUAL(segment1.max, 1);

    TEST_ASSERT_EQUAL(area1.min.x, -1);
    TEST_ASSERT_EQUAL(area1.min.y, -2);
    TEST_ASSERT_EQUAL(area1.max.x, 1);
    TEST_ASSERT_EQUAL(area1.max.y, 2);

    TEST_ASSERT_EQUAL(volume1.min.x, -1);
    TEST_ASSERT_EQUAL(volume1.min.y, -2);
    TEST_ASSERT_EQUAL(volume1.min.z, -3);
    TEST_ASSERT_EQUAL(volume1.max.x, 1);
    TEST_ASSERT_EQUAL(volume1.max.y, 2);
    TEST_ASSERT_EQUAL(volume1.max.z, 3);
}

void test_constraints_constructors2() {
    ConstraintSegment segment1(-1, 1);
    ConstraintArea area1(Vector2D(-1, -2), Vector2D(1, 2));
    ConstraintVolume volume1(Vector3D(-1, -2, -3), Vector3D(1, 2, 3));

    ConstraintSegment segment2(segment1);
    ConstraintArea area2(area1);
    ConstraintVolume volume2(volume1);

    TEST_ASSERT_EQUAL(segment2.min, -1);
    TEST_ASSERT_EQUAL(segment2.max, 1);

    TEST_ASSERT_EQUAL(area2.min.x, -1);
    TEST_ASSERT_EQUAL(area2.min.y, -2);
    TEST_ASSERT_EQUAL(area2.max.x, 1);
    TEST_ASSERT_EQUAL(area2.max.y, 2);

    TEST_ASSERT_EQUAL(volume2.min.x, -1);
    TEST_ASSERT_EQUAL(volume2.min.y, -2);
    TEST_ASSERT_EQUAL(volume2.min.z, -3);
    TEST_ASSERT_EQUAL(volume2.max.x, 1);
    TEST_ASSERT_EQUAL(volume2.max.y, 2);
    TEST_ASSERT_EQUAL(volume2.max.z, 3);
}

void test_constraints_constructors3() {
    ConstraintSegment segment1(-1, 1);
    ConstraintArea area1(Vector2D(-1, -2), Vector2D(1, 2));
    ConstraintVolume volume1(Vector3D(-1, -2, -3), Vector3D(1, 2, 3));

    ConstraintSegment segment3;
    ConstraintArea area3;
    ConstraintVolume volume3;

    segment3 = segment1;
    area3 = area1;
    volume3 = volume1;

    TEST_ASSERT_EQUAL(segment3.min, -1);
    TEST_ASSERT_EQUAL(segment3.max, 1);

    TEST_ASSERT_EQUAL(area3.min.x, -1);
    TEST_ASSERT_EQUAL(area3.min.y, -2);
    TEST_ASSERT_EQUAL(area3.max.x, 1);
    TEST_ASSERT_EQUAL(area3.max.y, 2);

    TEST_ASSERT_EQUAL(volume3.min.x, -1);
    TEST_ASSERT_EQUAL(volume3.min.y, -2);
    TEST_ASSERT_EQUAL(volume3.min.z, -3);
    TEST_ASSERT_EQUAL(volume3.max.x, 1);
    TEST_ASSERT_EQUAL(volume3.max.y, 2);
    TEST_ASSERT_EQUAL(volume3.max.z, 3);
}

void test_constraints_in_s() {
    ConstraintSegment segment(-1, 1);

    TEST_ASSERT_EQUAL(segment.in(-2), false);
}

void test_constraints_in_a() {
    ConstraintArea area(Vector2D(-1, -2), Vector2D(1, 2));

    TEST_ASSERT_EQUAL(area.in(Vector2D(-10, 0)), false);
    TEST_ASSERT_EQUAL(area.in(Vector2D(10, 0)), false);
    TEST_ASSERT_EQUAL(area.in(Vector2D(0, -10)), false);
    TEST_ASSERT_EQUAL(area.in(Vector2D(0, 10)), false);
    TEST_ASSERT_EQUAL(area.in(Vector2D(-0.5, 1)), true);
}

void test_constraints_in_v() {
    ConstraintVolume volume(Vector3D(-1, -2, -3), Vector3D(1, 2, 3));

    TEST_ASSERT_EQUAL(volume.in(Vector3D(-10, 0, 0)), false);
    TEST_ASSERT_EQUAL(volume.in(Vector3D(10, 0, 0)), false);
    TEST_ASSERT_EQUAL(volume.in(Vector3D(0, -10, 0)), false);
    TEST_ASSERT_EQUAL(volume.in(Vector3D(0, 10, 0)), false);
    TEST_ASSERT_EQUAL(volume.in(Vector3D(0, 0, -10)), false);
    TEST_ASSERT_EQUAL(volume.in(Vector3D(0, 0, 10)), false);
    TEST_ASSERT_EQUAL(volume.in(Vector3D(.5, 1, -2)), true);
}

void test_constraints_length() {
    ConstraintSegment segment(-1, 1);
    TEST_ASSERT_EQUAL(segment.length(), 2);
}

void test_constraints_area() {
    ConstraintArea area(Vector2D(-1, -2), Vector2D(1, 2));
    TEST_ASSERT_EQUAL(area.area(), 2 * 4);
}

void test_constraints_volume() {
    ConstraintVolume volume(Vector3D(-1, -2, -3), Vector3D(1, 2, 3));
    TEST_ASSERT_EQUAL(volume.volume(), 2 * 4 * 6);
}

void test_constraints_limit_s() {
    ConstraintSegment segment(-1, 1);
    TEST_ASSERT_EQUAL(segment.limit(-5), -1);
    TEST_ASSERT_EQUAL(segment.limit(7), 1);
    TEST_ASSERT_EQUAL(segment.limit(-.5), -.5);
}
void test_constraints_limit_a() {
    ConstraintArea area(Vector2D(-1, -2), Vector2D(1, 2));
    Vector2D l, r, u, d, m;
    l = area.limit(Vector2D(-10, 1));
    r = area.limit(Vector2D(10, 1));
    u = area.limit(Vector2D(0, 10));
    d = area.limit(Vector2D(0, -10));
    m = area.limit(Vector2D(.5, 1));

    TEST_ASSERT_EQUAL(l.x, -1);
    TEST_ASSERT_EQUAL(l.y, 1);
    TEST_ASSERT_EQUAL(r.x, 1);
    TEST_ASSERT_EQUAL(r.y, 1);
    TEST_ASSERT_EQUAL(u.x, 0);
    TEST_ASSERT_EQUAL(u.y, 2);
    TEST_ASSERT_EQUAL(d.x, 0);
    TEST_ASSERT_EQUAL(d.y, -2);
    TEST_ASSERT_EQUAL(m.x, .5);
    TEST_ASSERT_EQUAL(m.y, 1);
}
void test_constraints_limit_v() {
    ConstraintVolume volume(Vector3D(-1, -2, -3), Vector3D(1, 2, 3));
    Vector3D l, r, u, d, f, n, m;
    l = volume.limit(Vector3D(-10, 1, 1));
    r = volume.limit(Vector3D(10, 1, 1));
    u = volume.limit(Vector3D(0, 10, 1));
    d = volume.limit(Vector3D(0, -10, 1));
    f = volume.limit(Vector3D(.5, 1, 10));
    n = volume.limit(Vector3D(.5, 1, -10));
    m = volume.limit(Vector3D(.5, 1, 1.5));

    TEST_ASSERT_EQUAL(l.x, -1);
    TEST_ASSERT_EQUAL(l.y, 1);
    TEST_ASSERT_EQUAL(l.z, 1);
    TEST_ASSERT_EQUAL(r.x, 1);
    TEST_ASSERT_EQUAL(r.y, 1);
    TEST_ASSERT_EQUAL(r.z, 1);
    TEST_ASSERT_EQUAL(u.x, 0);
    TEST_ASSERT_EQUAL(u.y, 2);
    TEST_ASSERT_EQUAL(u.z, 1);
    TEST_ASSERT_EQUAL(d.x, 0);
    TEST_ASSERT_EQUAL(d.y, -2);
    TEST_ASSERT_EQUAL(d.z, 1);
    TEST_ASSERT_EQUAL(f.x, .5);
    TEST_ASSERT_EQUAL(f.y, 1);
    TEST_ASSERT_EQUAL(f.z, 3);
    TEST_ASSERT_EQUAL(n.x, .5);
    TEST_ASSERT_EQUAL(n.y, 1);
    TEST_ASSERT_EQUAL(n.z, -3);
    TEST_ASSERT_EQUAL(m.x, .5);
    TEST_ASSERT_EQUAL(m.y, 1);
    TEST_ASSERT_EQUAL(m.z, 1.5);
}

void test_constraints_middle_s() {
    ConstraintSegment segment(-1, 1);
    TEST_ASSERT_EQUAL(segment.middle(), 0);
}
void test_constraints_middle_a() {
    ConstraintArea area(Vector2D(-1, -3), Vector2D(2, 2));
    Vector2D middle = area.middle();
    TEST_ASSERT_EQUAL(middle.x, .5);
    TEST_ASSERT_EQUAL(middle.y, -.5);
}
void test_constraints_middle_v() {
    ConstraintVolume volume(Vector3D(-4, -10, -1), Vector3D(1, 2, 3));
    Vector3D middle = volume.middle();
    TEST_ASSERT_EQUAL(middle.x, -1.5);
    TEST_ASSERT_EQUAL(middle.y, -4);
    TEST_ASSERT_EQUAL(middle.z, 1);
}
