package configure

import (
	"github.com/robolaunch/robot-operator/internal/label"
	corev1 "k8s.io/api/core/v1"
)

func (cfg *PodSpecConfigInjector) InjectTimezone(podSpec *corev1.PodSpec, node corev1.Node) {
	cfg.placeTimezone(podSpec, node)
}

func (cfg *PodSpecConfigInjector) placeTimezone(podSpec *corev1.PodSpec, node corev1.Node) {

	timezone := label.GetTimezone(&node)
	if timezone.Continent == "" || timezone.City == "" {
		return
	}

	environmentVariables := []corev1.EnvVar{
		{
			Name:  "TZ",
			Value: timezone.Continent + "/" + timezone.City,
		},
	}

	for k, container := range podSpec.Containers {
		container.Env = append(container.Env, environmentVariables...)
		podSpec.Containers[k] = container
	}

}
