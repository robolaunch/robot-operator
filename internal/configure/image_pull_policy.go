package configure

import (
	v1 "k8s.io/api/batch/v1"
	corev1 "k8s.io/api/core/v1"
)

func (cfg *PodConfigInjector) InjectImagePullPolicy(pod *corev1.Pod) *corev1.Pod {

	cfg.placeImagePullPolicy(pod)

	return pod
}

func (cfg *JobConfigInjector) InjectImagePullPolicy(job *v1.Job) *v1.Job {

	cfg.placeImagePullPolicy(job)

	return job
}

func (cfg *PodConfigInjector) placeImagePullPolicy(pod *corev1.Pod) {

	for k, container := range pod.Spec.Containers {
		container.ImagePullPolicy = corev1.PullIfNotPresent
		pod.Spec.Containers[k] = container
	}

	for k, container := range pod.Spec.InitContainers {
		container.ImagePullPolicy = corev1.PullIfNotPresent
		pod.Spec.InitContainers[k] = container
	}

}

func (cfg *JobConfigInjector) placeImagePullPolicy(job *v1.Job) {

	podSpec := job.Spec.Template.Spec

	for k, container := range podSpec.Containers {
		container.ImagePullPolicy = corev1.PullIfNotPresent
		podSpec.Containers[k] = container
	}

	for k, container := range podSpec.InitContainers {
		container.ImagePullPolicy = corev1.PullIfNotPresent
		podSpec.InitContainers[k] = container
	}

	job.Spec.Template.Spec = podSpec
}
