package configure

import (
	"strconv"

	corev1 "k8s.io/api/core/v1"
)

func (cfg *PodSpecConfigInjector) InjectROSDomainID(podSpec *corev1.PodSpec, domainID int) {
	cfg.placeROSDomainIDEnvironmentVariables(podSpec, domainID)
}

func (cfg *PodSpecConfigInjector) placeROSDomainIDEnvironmentVariables(podSpec *corev1.PodSpec, domainID int) {

	environmentVariables := []corev1.EnvVar{
		{
			Name:  "ROS_DOMAIN_ID",
			Value: strconv.Itoa(domainID),
		},
	}

	for k, container := range podSpec.Containers {
		container.Env = append(container.Env, environmentVariables...)
		podSpec.Containers[k] = container
	}

}
