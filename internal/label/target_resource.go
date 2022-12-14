package label

import (
	"github.com/robolaunch/robot-operator/internal"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

func GetTargetRobot(obj metav1.Object) string {
	labels := obj.GetLabels()

	if targetRobot, ok := labels[internal.TARGET_ROBOT]; ok {
		return targetRobot
	}

	return ""
}

func GetTargetRobotVDI(obj metav1.Object) string {
	labels := obj.GetLabels()

	if targetRobotVDI, ok := labels[internal.TARGET_VDI]; ok {
		return targetRobotVDI
	}

	return ""
}
