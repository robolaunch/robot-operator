package configure

import (
	"strconv"

	corev1 "k8s.io/api/core/v1"
)

func (cfg *PodConfigInjector) InjectROSDomainID(pod *corev1.Pod, domainID int) *corev1.Pod {

	cfg.placeROSDomainIDEnvironmentVariables(pod, domainID)

	return pod
}

func (cfg *PodConfigInjector) placeROSDomainIDEnvironmentVariables(pod *corev1.Pod, domainID int) {

	environmentVariables := []corev1.EnvVar{
		{
			Name:  "ROS_DOMAIN_ID",
			Value: strconv.Itoa(domainID),
		},
	}

	for k, container := range pod.Spec.Containers {
		container.Env = append(container.Env, environmentVariables...)
		pod.Spec.Containers[k] = container
	}

}
