package configure

import (
	"github.com/robolaunch/robot-operator/internal"
	corev1 "k8s.io/api/core/v1"
)

func (cfg *PodSpecConfigInjector) InjectRMWImplementationConfiguration(podSpec *corev1.PodSpec, rmwImplementation string) {
	environmentVariables := []corev1.EnvVar{
		internal.Env("RMW_IMPLEMENTATION", rmwImplementation),
	}

	for k, container := range podSpec.Containers {
		container.Env = append(container.Env, environmentVariables...)
		podSpec.Containers[k] = container
	}
}
