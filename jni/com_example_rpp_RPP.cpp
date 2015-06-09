#include <jni.h>
#include "librpp/librpp.h"
#include "com_example_rpp_RPP.h"

#include <android/log.h>
#define LOG_TAG "fyy"
#define LOGI(fmt, args...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, fmt, ##args)
#define LOGD(fmt, args...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, fmt, ##args)
#define LOGE(fmt, args...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, fmt, ##args)

void Java_com_camera_pose_estimate_algorithm_rpp_RPP_estimate(JNIEnv * env, jclass clazz,
		jdoubleArray jPrincipalPoint, jdoubleArray jFocalLength,
		jdoubleArray jModelPoints, jdoubleArray jImagePoints, jint jPointsNum,
		jobjectArray jR_init, jboolean jEstimateByR_init, jdouble jEpsilon,
		jdouble jTolerance, jint jMaxIiterations, jobject result) {
	rpp_float resultError;
	rpp_mat resultR;
	rpp_vec resultT;

	jdouble* cTmp = env->GetDoubleArrayElements(jPrincipalPoint, 0);
	rpp_float cc[2] = { cTmp[0], cTmp[1] };
	env->ReleaseDoubleArrayElements(jPrincipalPoint, cTmp, 0);
	cTmp = env->GetDoubleArrayElements(jFocalLength, 0);
	rpp_float fc[2] = { cTmp[0], cTmp[1] };
	env->ReleaseDoubleArrayElements(jFocalLength, cTmp, 0);

	cTmp = env->GetDoubleArrayElements(jModelPoints, 0);
	rpp_vec modelPoints[4] = {
	//
			{ cTmp[0], cTmp[1], cTmp[2] }, //
			{ cTmp[3], cTmp[4], cTmp[5] }, //
			{ cTmp[6], cTmp[7], cTmp[8] }, //
			{ cTmp[9], cTmp[10], cTmp[11] }, };
	env->ReleaseDoubleArrayElements(jModelPoints, cTmp, 0);
	cTmp = env->GetDoubleArrayElements(jImagePoints, 0);
	rpp_vec imagePoints[4] = {
	//
			{ cTmp[0], cTmp[1], cTmp[2] }, //
			{ cTmp[3], cTmp[4], cTmp[5] }, //
			{ cTmp[6], cTmp[7], cTmp[8] }, //
			{ cTmp[9], cTmp[10], cTmp[11] }, };
//	rpp_vec imagePoints[4] = {
//	//
//			{ -cTmp[0], -cTmp[1], cTmp[2] }, //
//			{ -cTmp[3], -cTmp[4], cTmp[5] }, //
//			{ -cTmp[6], -cTmp[7], cTmp[8] }, //
//			{ -cTmp[9], -cTmp[10], cTmp[11] }, };
	env->ReleaseDoubleArrayElements(jImagePoints, cTmp, 0);

	int pointsNum = 4;/*ignore jPointsNum , always four*/
	rpp_mat R_init;/*not use , ignore jR_init*/
	bool estimate_R_init = true;/*ignore jEstimateByR_init , always true*/

	robustPlanarPose(resultError, resultR, resultT, cc, fc, modelPoints,
			imagePoints, pointsNum, R_init, estimate_R_init, jEpsilon,
			jTolerance, jMaxIiterations);

	// 1.获取PoseResult类的Class引用
	jclass poseResultClass = env->GetObjectClass(result);
	// 2. 获取PoseResult类的属性ID
	jfieldID errorID = env->GetFieldID(poseResultClass, "error", "D");
	jfieldID translationID = env->GetFieldID(poseResultClass, "translation",
			"[D");
	jfieldID rotationID = env->GetFieldID(poseResultClass, "rotation", "[[D");
	//3.赋值
	env->SetDoubleField(result, errorID, resultError);
	jdoubleArray translationArray = (jdoubleArray) env->GetObjectField(result,
			translationID);
	env->SetDoubleArrayRegion(translationArray, 0, 3, resultT);
	jobjectArray rotationArray = (jobjectArray) env->GetObjectField(result,
			rotationID);
	jdoubleArray rotationRow0 = (jdoubleArray) env->GetObjectArrayElement(
			rotationArray, 0);
	jdoubleArray rotationRow1 = (jdoubleArray) env->GetObjectArrayElement(
			rotationArray, 1);
	jdoubleArray rotationRow2 = (jdoubleArray) env->GetObjectArrayElement(
			rotationArray, 2);
	env->SetDoubleArrayRegion(rotationRow0, 0, 3, resultR[0]);
	env->SetDoubleArrayRegion(rotationRow1, 0, 3, resultR[1]);
	env->SetDoubleArrayRegion(rotationRow2, 0, 3, resultR[2]);
	//4.删除局部引用
	env->DeleteLocalRef(poseResultClass);
	env->DeleteLocalRef(translationArray);
	env->DeleteLocalRef(rotationArray);
	env->DeleteLocalRef(rotationRow0);
	env->DeleteLocalRef(rotationRow1);
	env->DeleteLocalRef(rotationRow2);
}
