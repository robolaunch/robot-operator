package resources

import (
	"strconv"

	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/configure"
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
			RestartPolicy: corev1.RestartPolicyNever,
		},
	}

	if metricsExporter.Spec.GPU.Track {
		pod.Spec.Containers = append(pod.Spec.Containers, corev1.Container{
			Name:    "gpu-util",
			Image:   "robolaunchio/custom-metrics-dev:v0.0.01",
			Command: internal.Bash("./gpu-util.sh"),
			Env: []corev1.EnvVar{
				internal.Env("GPU_LATENCY", strconv.Itoa(metricsExporter.Spec.GPU.Interval)),
			},
			Resources: corev1.ResourceRequirements{
				Limits: getResourceLimits(robotv1alpha1.Resources{
					GPUCore: 0,
				}),
			},
			SecurityContext: &corev1.SecurityContext{
				Privileged: &privileged,
			},
		})
	}

	configure.InjectRuntimeClassWithoutRobot(&pod, node)

	return &pod
}
