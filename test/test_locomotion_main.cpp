#if  defined(UNIT_TEST)

#if defined(ARDUINO)
#include <Arduino.h>

template<typename T> void print_matrix(const T& m, size_t N, size_t M, const char * name) {
    Serial.print("matrix ");
    Serial.println(name);
    for (int row = 0; row < N; row++) {
        for (int col=0;col<M;col++) {
            Serial.print(" ");
            Serial.print(m.val(row, col));
        }
        Serial.println();
    }
}

template<typename T> void print_vector(const T& v, size_t N, const char * name) {
    Serial.print("vector ");
    Serial.println(name);
    for (int col=0;col<N;col++) {
        Serial.print(" ");
        Serial.print(v.data()[col]);
    }
    Serial.println();
}
#else
#include <string.h>
#include <limits.h>
typedef unsigned char byte;

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

#include <unity.h>

#include "math_utils.h"
#include "kinematics/PlanarKinematics.h"

using namespace Locomotion;


#include "math_utils/test_math_utils_main.hpp"
#include "kinematics/test_kinematics_main.hpp"

#define _RUN_TEST(a) \
    clean_stack(); \
    RUN_TEST(a);

void clean_stack() {
    byte arr[2048];
    memset(arr, 0xAA, sizeof(arr));
}

#if defined(ARDUINO)
void setup() {
    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);

    UNITY_BEGIN();

    MATH_UTILS_TESTS;
    KINEMATICS_TESTS;

    UNITY_END();

}

void loop() {
}
#else

int main(int argc, char **argv) {
    UNITY_BEGIN();

    MATH_UTILS_TESTS;
    KINEMATICS_TESTS;

    UNITY_END();
}

#endif

#endif