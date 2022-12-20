package configure

import (
	"github.com/robolaunch/robot-operator/internal"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
)

func InjectGenericEnvironmentVariablesForPodSpec(podSpec *corev1.PodSpec, robot robotv1alpha1.Robot) *corev1.PodSpec {

	for key, cont := range podSpec.Containers {
		cont.Env = append(cont.Env, internal.Env("WORKSPACES_PATH", robot.Spec.WorkspacesPath))
		podSpec.Containers[key] = cont
	}

	return podSpec
}

func InjectGenericEnvironmentVariables(pod *corev1.Pod, robot robotv1alpha1.Robot) *corev1.Pod {

	for key, cont := range pod.Spec.Containers {
		cont.Env = append(cont.Env, internal.Env("WORKSPACES_PATH", robot.Spec.WorkspacesPath))
		pod.Spec.Containers[key] = cont
	}

	return pod
}
