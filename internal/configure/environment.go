package configure

import (
	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	"github.com/robolaunch/robot-operator/internal"
	corev1 "k8s.io/api/core/v1"
)

func InjectGenericEnvironmentVariables(podSpec *corev1.PodSpec, robot robotv1alpha1.Robot) *corev1.PodSpec {

	for key, cont := range podSpec.Containers {
		cont.Env = append(cont.Env, internal.Env("WORKSPACES_PATH", robot.Spec.WorkspacesPath))
		podSpec.Containers[key] = cont
	}

	return podSpec
}
