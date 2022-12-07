package spawn

import (
	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/types"
)

func GetBridgePod(rosbridge *robotv1alpha1.ROSBridge, podNamespacedName *types.NamespacedName) *corev1.Pod {

	return &corev1.Pod{}
}

func GetBridgeService(rosbridge *robotv1alpha1.ROSBridge, svcNamespacedName *types.NamespacedName) *corev1.Service {

	return &corev1.Service{}
}
