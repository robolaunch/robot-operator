package configure

import (
	"github.com/robolaunch/robot-operator/internal/label"
	batchv1 "k8s.io/api/batch/v1"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

func (cfg *PodConfigInjector) SchedulePod(pod *corev1.Pod, obj metav1.Object) *corev1.Pod {

	pod.Spec.NodeSelector = label.GetTenancyMap(obj)

	return pod
}

func (cfg *JobConfigInjector) SchedulePod(job *batchv1.Job, obj metav1.Object) *batchv1.Job {

	podSpec := job.Spec.Template.Spec

	podSpec.NodeSelector = label.GetTenancyMap(obj)

	job.Spec.Template.Spec = podSpec

	return job
}
