package configure

import (
	"github.com/robolaunch/robot-operator/internal"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
)

func InjectRMWImplementationConfiguration(pod *corev1.Pod, robot robotv1alpha1.Robot) *corev1.Pod {

	placeRMWImplementationEnvironmentVariables(pod, robot)

	return pod
}

func placeRMWImplementationEnvironmentVariables(pod *corev1.Pod, robot robotv1alpha1.Robot) {

	environmentVariables := []corev1.EnvVar{
		internal.Env("RMW_IMPLEMENTATION", string(robot.Spec.RMWImplementation)),
	}

	for k, container := range pod.Spec.Containers {
		container.Env = append(container.Env, environmentVariables...)
		pod.Spec.Containers[k] = container
	}

}
