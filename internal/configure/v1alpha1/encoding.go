package configure

import (
	"github.com/robolaunch/robot-operator/internal"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
)

func (cfg *PodConfigInjector) InjectEncodingOption(pod *corev1.Pod, robot robotv1alpha1.Robot) *corev1.Pod {

	environmentVariables := []corev1.EnvVar{
		internal.Env("NEKO_HWENC", "nvenc"),
	}

	for k, container := range pod.Spec.Containers {
		container.Env = append(container.Env, environmentVariables...)
		pod.Spec.Containers[k] = container
	}

	return pod
}
