package configure

import (
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

func (cfg *PodSpecConfigInjector) InjectRemoteConfigurations(podSpec *corev1.PodSpec, obj metav1.Object) *corev1.PodSpec {

	podSpec.Hostname = obj.GetName()
	podSpec.Subdomain = obj.GetName()

	return podSpec
}

func (cfg *ServiceSpecConfigInjector) InjectRemoteConfigurations(serviceSpec *corev1.ServiceSpec) *corev1.ServiceSpec {

	serviceSpec.Type = ""
	serviceSpec.ClusterIP = "None"

	return serviceSpec
}
