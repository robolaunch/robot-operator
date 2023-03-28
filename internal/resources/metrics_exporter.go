package resources

import (
	"github.com/robolaunch/robot-operator/internal"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
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
			Containers: []corev1.Container{
				{
					Name:    "gpu-util",
					Image:   "robolaunchio/custom-metrics-dev:v0.0.01",
					Command: internal.Bash("./gpu-util.sh"),
					Env: []corev1.EnvVar{
						internal.Env("GPU_LATENCY", "3"),
					},
					SecurityContext: &corev1.SecurityContext{
						Privileged: &privileged,
					},
				},
			},

			RestartPolicy: corev1.RestartPolicyNever,
		},
	}

	return &pod
}
