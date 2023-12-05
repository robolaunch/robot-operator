package configure

import (
	"github.com/robolaunch/robot-operator/internal/label"
	batchv1 "k8s.io/api/batch/v1"
	corev1 "k8s.io/api/core/v1"
)

func (cfg *PodConfigInjector) InjectTimezone(pod *corev1.Pod, node corev1.Node) *corev1.Pod {

	cfg.placeTimezone(pod, node)

	return pod
}

func (cfg *JobConfigInjector) InjectTimezone(job *batchv1.Job, node corev1.Node) *batchv1.Job {

	cfg.placeTimezone(job, node)

	return job
}

func (cfg *PodConfigInjector) placeTimezone(pod *corev1.Pod, node corev1.Node) {

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

	for k, container := range pod.Spec.Containers {
		container.Env = append(container.Env, environmentVariables...)
		pod.Spec.Containers[k] = container
	}

}

func (cfg *JobConfigInjector) placeTimezone(job *batchv1.Job, node corev1.Node) {

	podSpec := job.Spec.Template.Spec

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

	job.Spec.Template.Spec = podSpec

}
