package configure

import (
	"github.com/robolaunch/robot-operator/internal"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	batchv1 "k8s.io/api/batch/v1"
	corev1 "k8s.io/api/core/v1"
)

func (cfg *PodConfigInjector) InjectRMWImplementationConfiguration(pod *corev1.Pod, robot robotv1alpha1.Robot) *corev1.Pod {

	environmentVariables := []corev1.EnvVar{
		internal.Env("RMW_IMPLEMENTATION", string(robot.Spec.RobotConfig.RMWImplementation)),
	}

	for k, container := range pod.Spec.Containers {
		container.Env = append(container.Env, environmentVariables...)
		pod.Spec.Containers[k] = container
	}

	return pod
}

func (cfg *JobConfigInjector) InjectRMWImplementationConfiguration(job *batchv1.Job, robot robotv1alpha1.Robot) *batchv1.Job {

	podSpec := job.Spec.Template.Spec

	environmentVariables := []corev1.EnvVar{
		internal.Env("RMW_IMPLEMENTATION", string(robot.Spec.RobotConfig.RMWImplementation)),
	}

	for k, container := range podSpec.Containers {
		container.Env = append(container.Env, environmentVariables...)
		podSpec.Containers[k] = container
	}

	job.Spec.Template.Spec = podSpec

	return job
}
