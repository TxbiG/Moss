
#include <Moss/Platform/android/platform_android.h>

static JNIEnv* moss_env = NULL;
static jobject moss_activity = NULL;



static inline void Moss_PlayHapticEffect(const Moss_HapticEffect *effect) {
    if (moss_env && moss_activity) {
        jclass activityClass = (*moss_env)->GetObjectClass(moss_env, moss_activity);
        jmethodID getSystemService = (*moss_env)->GetMethodID(moss_env, activityClass, "getSystemService", "(Ljava/lang/String;)Ljava/lang/Object;");
        jstring serviceName = (*moss_env)->NewStringUTF(moss_env, "vibrator");
        jobject vibrator = (*moss_env)->CallObjectMethod(moss_env, moss_activity, getSystemService, serviceName);
        (*moss_env)->DeleteLocalRef(moss_env, serviceName);

        if (vibrator) {
            jclass vibClass = (*moss_env)->GetObjectClass(moss_env, vibrator);
            jmethodID vibrate = (*moss_env)->GetMethodID(moss_env, vibClass, "vibrate", "(J)V");
            (*moss_env)->CallVoidMethod(moss_env, vibrator, vibrate, (jlong)effect->leftright.length);
        }
    }

}
