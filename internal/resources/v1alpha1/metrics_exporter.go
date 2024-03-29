package resources

import (
	"strconv"

	"github.com/robolaunch/robot-operator/internal"
	configure "github.com/robolaunch/robot-operator/internal/configure/v1alpha1"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	rbacv1 "k8s.io/api/rbac/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
)

var metricsPatcherImage = "robolaunchio/custom-metrics-patcher-dev:focal-v1.24.10-0.1.3"

func GetMetricsExporterPod(metricsExporter *robotv1alpha1.MetricsExporter, podNamespacedName *types.NamespacedName, node corev1.Node) *corev1.Pod {

	cfg := configure.PodConfigInjector{}

	pod := corev1.Pod{
		ObjectMeta: metav1.ObjectMeta{
			Name:      podNamespacedName.Name,
			Namespace: podNamespacedName.Namespace,
		},
		Spec: corev1.PodSpec{
			RestartPolicy:      corev1.RestartPolicyNever,
			ServiceAccountName: metricsExporter.GetMetricsExporterServiceAccountMetadata().Name,
			HostNetwork:        true,
			DNSPolicy:          corev1.DNSClusterFirstWithHostNet,
			Volumes: []corev1.Volume{
				{
					Name: "fstab",
					VolumeSource: corev1.VolumeSource{
						HostPath: &corev1.HostPathVolumeSource{
							Path: "/etc/fstab",
						},
					},
				},
			},
		},
	}

	if metricsExporter.Spec.GPU.Track && metricsExporter.Status.Usage.GPUDeviceStatuses.DCGMEndpoint != "" {
		pod.Spec.Containers = append(pod.Spec.Containers, corev1.Container{
			Name:    "dcgm-gpu-util",
			Image:   metricsPatcherImage,
			Command: internal.Bash("./dcgm-gpu-util.sh"),
			Env: []corev1.EnvVar{
				internal.Env("METRICS_EXPORTER_NAME", metricsExporter.Name),
				internal.Env("METRICS_EXPORTER_NAMESPACE", metricsExporter.Namespace),
				internal.Env("INTERVAL", strconv.Itoa(metricsExporter.Spec.GPU.Interval)),
				internal.Env("DCGM_METRICS_ENDPOINT", metricsExporter.Status.Usage.GPUDeviceStatuses.DCGMEndpoint),
			},
		})
	}

	if metricsExporter.Spec.Network.Track {
		pod.Spec.Containers = append(pod.Spec.Containers, corev1.Container{
			Name:    "network-load",
			Image:   metricsPatcherImage,
			Command: internal.Bash("./network-load.sh"),
			Env: []corev1.EnvVar{
				internal.Env("METRICS_EXPORTER_NAME", metricsExporter.Name),
				internal.Env("METRICS_EXPORTER_NAMESPACE", metricsExporter.Namespace),
				internal.Env("INTERVAL", strconv.Itoa(metricsExporter.Spec.Network.Interval)),
				internal.Env("NETWORK_INTERFACES", formatNetworkInterfaces(metricsExporter.Spec.Network.Interfaces)),
			},
		})
	}

	if metricsExporter.Spec.Storage.Track {
		privileged := true
		pod.Spec.Containers = append(pod.Spec.Containers, corev1.Container{
			Name:    "storage-usage",
			Image:   metricsPatcherImage,
			Command: internal.Bash("./storage-usage.sh"),
			Env: []corev1.EnvVar{
				internal.Env("METRICS_EXPORTER_NAME", metricsExporter.Name),
				internal.Env("METRICS_EXPORTER_NAMESPACE", metricsExporter.Namespace),
				internal.Env("INTERVAL", strconv.Itoa(metricsExporter.Spec.Network.Interval)),
			},
			SecurityContext: &corev1.SecurityContext{
				Privileged: &privileged,
			},
			VolumeMounts: []corev1.VolumeMount{
				{
					Name:      "fstab",
					MountPath: "/etc/fstab",
				},
			},
		})
	}

	cfg.InjectImagePullPolicy(&pod)
	cfg.InjectRuntimeClassForMetricsExporter(&pod, node)

	return &pod
}

func formatNetworkInterfaces(interfaces []string) string {
	formatted := ""
	for k, v := range interfaces {
		if k != 0 {
			formatted += ","
		}
		formatted += v
	}
	return formatted
}

func GetMetricsExporterRole(metricsExporter *robotv1alpha1.MetricsExporter, roleNamespacedName *types.NamespacedName) *rbacv1.Role {

	role := rbacv1.Role{
		ObjectMeta: metav1.ObjectMeta{
			Name:      roleNamespacedName.Name,
			Namespace: roleNamespacedName.Namespace,
		},
		Rules: []rbacv1.PolicyRule{
			{
				APIGroups: []string{
					metricsExporter.DeepCopy().GroupVersionKind().Group,
				},
				Resources: []string{"metricsexporters/status"},
				// get;update;patch
				Verbs: []string{
					"get",
					"update",
					"patch",
				},
			},
			{
				APIGroups: []string{
					metricsExporter.DeepCopy().GroupVersionKind().Group,
				},
				Resources: []string{"metricsexporters"},
				// get;update;patch
				Verbs: []string{
					"get",
					"update",
					"patch",
				},
			},
		},
	}

	return &role
}

func GetMetricsExporterRoleBinding(metricsExporter *robotv1alpha1.MetricsExporter, rbNamespacedName *types.NamespacedName) *rbacv1.RoleBinding {

	rb := rbacv1.RoleBinding{
		ObjectMeta: metav1.ObjectMeta{
			Name:      rbNamespacedName.Name,
			Namespace: rbNamespacedName.Namespace,
		},
		RoleRef: rbacv1.RoleRef{
			Kind:     "Role",
			APIGroup: "rbac.authorization.k8s.io",
			Name:     metricsExporter.GetMetricsExporterRoleMetadata().Name,
		},
		Subjects: []rbacv1.Subject{
			{
				Kind:      "ServiceAccount",
				Name:      metricsExporter.GetMetricsExporterServiceAccountMetadata().Name,
				Namespace: metricsExporter.Namespace,
			},
		},
	}

	return &rb
}

func GetMetricsExporterServiceAccount(metricsExporter *robotv1alpha1.MetricsExporter, saNamespacedName *types.NamespacedName) *corev1.ServiceAccount {

	sa := corev1.ServiceAccount{
		ObjectMeta: metav1.ObjectMeta{
			Name:      metricsExporter.GetMetricsExporterServiceAccountMetadata().Name,
			Namespace: metricsExporter.GetMetricsExporterServiceAccountMetadata().Namespace,
		},
	}

	return &sa
}
