#include <jni.h>
#include <string>
#include "Eigen/Geometry"

extern "C" JNIEXPORT jstring JNICALL
Java_com_example_myapplication3_MainActivity_stringFromJNI(
        JNIEnv* env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}
