package resources

import (
	"strconv"

	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/configure"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	rbacv1 "k8s.io/api/rbac/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
)

func GetMetricsExporterPod(metricsExporter *robotv1alpha1.MetricsExporter, podNamespacedName *types.NamespacedName, node corev1.Node) *corev1.Pod {

	privileged := true

	pod := corev1.Pod{
		ObjectMeta: metav1.ObjectMeta{
			Name:      podNamespacedName.Name,
			Namespace: podNamespacedName.Namespace,
		},
		Spec: corev1.PodSpec{
			RestartPolicy: corev1.RestartPolicyNever,
		},
	}

	if metricsExporter.Spec.GPU.Track {
		pod.Spec.Containers = append(pod.Spec.Containers, corev1.Container{
			Name:    "gpu-util",
			Image:   "robolaunchio/custom-metrics-dev:v0.0.02",
			Command: internal.Bash("./gpu-util.sh"),
			Env: []corev1.EnvVar{
				internal.Env("GPU_LATENCY", strconv.Itoa(metricsExporter.Spec.GPU.Interval)),
			},
			Resources: corev1.ResourceRequirements{
				Limits: getResourceLimits(robotv1alpha1.Resources{
					GPUCore: 1,
				}),
			},
			SecurityContext: &corev1.SecurityContext{
				Privileged: &privileged,
			},
		})
	}

	configure.InjectRuntimeClassForMetricsExporter(&pod, node)

	return &pod
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
