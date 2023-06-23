package configure

import (
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
)

// It should be applied ONLY in physical instances.
func InjectRemoteConfigurationsForPod(pod *corev1.Pod, robotIDE robotv1alpha1.RobotIDE) *corev1.Pod {

	pod.Spec.Hostname = robotIDE.Name
	pod.Spec.Subdomain = robotIDE.GetRobotIDEServiceMetadata().Name

	return pod
}

// It should be applied ONLY in physical instances.
func InjectRemoteConfigurationsForService(service *corev1.Service) *corev1.Service {

	service.Spec.Type = ""
	service.Spec.ClusterIP = "None"

	return service
}
