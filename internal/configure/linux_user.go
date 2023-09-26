package configure

import (
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	batchv1 "k8s.io/api/batch/v1"
	corev1 "k8s.io/api/core/v1"
)

func (cfg *PodConfigInjector) InjectLinuxUserAndGroup(pod *corev1.Pod, robot robotv1alpha1.Robot) *corev1.Pod {

	var user int64 = robot.Status.UID
	var group int64 = 3000

	for key, cont := range pod.Spec.Containers {
		cont.SecurityContext = &corev1.SecurityContext{
			RunAsUser:  &user,
			RunAsGroup: &group,
			Privileged: cont.SecurityContext.Privileged,
		}
		pod.Spec.Containers[key] = cont
	}

	return pod
}

func (cfg *JobConfigInjector) InjectLinuxUserAndGroup(job *batchv1.Job, robot robotv1alpha1.Robot) *batchv1.Job {

	podSpec := job.Spec.Template.Spec

	var user int64 = robot.Status.UID
	var group int64 = 3000

	for key, cont := range podSpec.Containers {
		cont.SecurityContext = &corev1.SecurityContext{
			RunAsUser:  &user,
			RunAsGroup: &group,
		}
		podSpec.Containers[key] = cont
	}

	job.Spec.Template.Spec = podSpec

	return job
}
