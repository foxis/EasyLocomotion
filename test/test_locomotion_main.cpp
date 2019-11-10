#if  defined(UNIT_TEST)

#if defined(ARDUINO)
#include <Arduino.h>

#define REAL_T float

template<typename T> void print_matrix(const T& m, size_t N, size_t M, const char * name) {
    Serial.print("matrix ");
    Serial.println(name);
    for (int row = 0; row < N; row++) {
        for (int col=0;col<M;col++) {
            Serial.print(" ");
            Serial.print(m.val(row, col), 6);
        }
        Serial.println();
    }
}

template<typename T> void print_vector(const T& v, size_t N, const char * name) {
    Serial.print("vector ");
    Serial.println(name);
    for (int col=0;col<N;col++) {
        Serial.print(" ");
        Serial.print(v.data()[col], 6);
    }
    Serial.println();
}

template<typename T> void print_val(const T a, const char * name) {
    Serial.print("value ");
    Serial.println(name);
    Serial.println(a, 6);
}

void print_text(const char * text) {
    Serial.print("Text: ");
    Serial.println(text);
}

void print_checkpoint_(int line) {
    Serial.print("Checkpoint line: ");
    Serial.println(line);
}

void print_duration_(int line, unsigned long duration) {
    Serial.print("Duration line: ");
    Serial.print(line);
    Serial.print(" ");
    Serial.print(duration);
    Serial.println(" us");
}

#define PP_CAT(a, b) PP_CAT_I(a, b)
#define PP_CAT_I(a, b) PP_CAT_II(~, a ## b)
#define PP_CAT_II(p, res) res

#define print_checkpoint print_checkpoint_(__LINE__);
#define print_duration(expression) print_duration__(expression, PP_CAT(now, __LINE__))
#define print_duration__(expression, now) \
    unsigned long now = micros(); \
    expression; \
    print_duration_(__LINE__, micros() - now);

#else
#include <string.h>
#include <limits.h>
typedef unsigned char byte;

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

#include <unity.h>

#include "math_utils.h"

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