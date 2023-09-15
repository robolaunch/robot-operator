package configure

import (
	batchv1 "k8s.io/api/batch/v1"
	corev1 "k8s.io/api/core/v1"
)

func InjectImagePullPolicyToPod(pod *corev1.Pod) *corev1.Pod {

	placeImagePullPolicyToPods(pod)

	return pod
}

func InjectImagePullPolicyToJob(job *batchv1.Job) *batchv1.Job {

	placeImagePullPolicyToJobs(job)

	return job
}

func placeImagePullPolicyToPods(pod *corev1.Pod) {

	for k, container := range pod.Spec.Containers {
		container.ImagePullPolicy = corev1.PullIfNotPresent
		pod.Spec.Containers[k] = container
	}

}

func placeImagePullPolicyToJobs(job *batchv1.Job) {

	for k, container := range job.Spec.Template.Spec.Containers {
		container.ImagePullPolicy = corev1.PullIfNotPresent
		job.Spec.Template.Spec.Containers[k] = container
	}

}
