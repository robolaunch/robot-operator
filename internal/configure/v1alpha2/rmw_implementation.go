package configure

import (
	"github.com/robolaunch/robot-operator/internal"
	corev1 "k8s.io/api/core/v1"
)

func (cfg *PodConfigInjector) InjectRMWImplementationConfiguration(pod *corev1.Pod, rmwImplementation string) *corev1.Pod {

	environmentVariables := []corev1.EnvVar{
		internal.Env("RMW_IMPLEMENTATION", rmwImplementation),
	}

	for k, container := range pod.Spec.Containers {
		container.Env = append(container.Env, environmentVariables...)
		pod.Spec.Containers[k] = container
	}

	return pod
}
