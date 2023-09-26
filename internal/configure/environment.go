package configure

import (
	"github.com/robolaunch/robot-operator/internal"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	batchv1 "k8s.io/api/batch/v1"
	corev1 "k8s.io/api/core/v1"
)

func (cfg *JobConfigInjector) InjectGenericEnvironmentVariables(job *batchv1.Job, robot robotv1alpha1.Robot) *batchv1.Job {

	podSpec := job.Spec.Template.Spec

	for key, cont := range podSpec.Containers {
		cont.Env = append(cont.Env, internal.Env("WORKSPACES_PATH", robot.Spec.WorkspaceManagerTemplate.WorkspacesPath))
		podSpec.Containers[key] = cont
	}

	for key, cont := range podSpec.InitContainers {
		cont.Env = append(cont.Env, internal.Env("WORKSPACES_PATH", robot.Spec.WorkspaceManagerTemplate.WorkspacesPath))
		podSpec.InitContainers[key] = cont
	}

	for cfgKey, cfg := range robot.Spec.AdditionalConfigs {
		if cfg.ConfigType == robotv1alpha1.AdditionalConfigTypeEnv {
			for key, cont := range podSpec.Containers {
				cont.Env = append(cont.Env, internal.Env(cfgKey, cfg.Value))
				podSpec.Containers[key] = cont
			}
		}
	}

	for cfgKey, cfg := range robot.Spec.AdditionalConfigs {
		if cfg.ConfigType == robotv1alpha1.AdditionalConfigTypeEnv {
			for key, cont := range podSpec.InitContainers {
				cont.Env = append(cont.Env, internal.Env(cfgKey, cfg.Value))
				podSpec.InitContainers[key] = cont
			}
		}
	}

	job.Spec.Template.Spec = podSpec

	return job
}

func (cfg *JobConfigInjector) InjectGenericRobotEnvironmentVariables(job *batchv1.Job, robot robotv1alpha1.Robot) *batchv1.Job {

	podSpec := job.Spec.Template.Spec

	for key, cont := range podSpec.Containers {
		cont.Env = append(cont.Env, internal.Env("ROS2_SETUP_PATH", "/opt/ros/"+string(robot.Spec.RobotConfig.Distributions[0])+"/setup.bash"))
		podSpec.Containers[key] = cont
	}

	for key, cont := range podSpec.InitContainers {
		cont.Env = append(cont.Env, internal.Env("ROS2_SETUP_PATH", "/opt/ros/"+string(robot.Spec.RobotConfig.Distributions[0])+"/setup.bash"))
		podSpec.InitContainers[key] = cont
	}

	job.Spec.Template.Spec = podSpec

	return job
}

func (cfg *PodConfigInjector) InjectGenericEnvironmentVariables(pod *corev1.Pod, robot robotv1alpha1.Robot) *corev1.Pod {

	for key, cont := range pod.Spec.Containers {
		cont.Env = append(cont.Env, internal.Env("WORKSPACES_PATH", robot.Spec.WorkspaceManagerTemplate.WorkspacesPath))
		pod.Spec.Containers[key] = cont
	}

	for key, cont := range pod.Spec.InitContainers {
		cont.Env = append(cont.Env, internal.Env("WORKSPACES_PATH", robot.Spec.WorkspaceManagerTemplate.WorkspacesPath))
		pod.Spec.InitContainers[key] = cont
	}

	for cfgKey, cfg := range robot.Spec.AdditionalConfigs {
		if cfg.ConfigType == robotv1alpha1.AdditionalConfigTypeEnv {
			for key, cont := range pod.Spec.Containers {
				cont.Env = append(cont.Env, internal.Env(cfgKey, cfg.Value))
				pod.Spec.Containers[key] = cont
			}
		}
	}

	for cfgKey, cfg := range robot.Spec.AdditionalConfigs {
		if cfg.ConfigType == robotv1alpha1.AdditionalConfigTypeEnv {
			for key, cont := range pod.Spec.InitContainers {
				cont.Env = append(cont.Env, internal.Env(cfgKey, cfg.Value))
				pod.Spec.InitContainers[key] = cont
			}
		}
	}

	return pod
}

func (cfg *PodConfigInjector) InjectGenericRobotEnvironmentVariables(pod *corev1.Pod, robot robotv1alpha1.Robot) *corev1.Pod {

	for key, cont := range pod.Spec.Containers {
		cont.Env = append(cont.Env, internal.Env("ROS2_SETUP_PATH", "/opt/ros/"+string(robot.Spec.RobotConfig.Distributions[0])+"/setup.bash"))
		pod.Spec.Containers[key] = cont
	}

	for key, cont := range pod.Spec.InitContainers {
		cont.Env = append(cont.Env, internal.Env("ROS2_SETUP_PATH", "/opt/ros/"+string(robot.Spec.RobotConfig.Distributions[0])+"/setup.bash"))
		pod.Spec.InitContainers[key] = cont
	}

	return pod
}

func (cfg *ContainerConfigInjector) InjectWorkspaceEnvironmentVariable(container *corev1.Container, robot robotv1alpha1.Robot, workspace string) *corev1.Container {
	container.Env = append(container.Env, internal.Env("WORKSPACE", robot.Spec.WorkspaceManagerTemplate.WorkspacesPath+"/"+workspace))
	return container
}

func (cfg *JobConfigInjector) InjectWorkspaceEnvironmentVariable(job *batchv1.Job, robot robotv1alpha1.Robot, workspace string) *batchv1.Job {

	podSpec := job.Spec.Template.Spec

	for key, cont := range podSpec.Containers {
		cont.Env = append(cont.Env, internal.Env("WORKSPACE", robot.Spec.WorkspaceManagerTemplate.WorkspacesPath+"/"+workspace))
		podSpec.Containers[key] = cont
	}

	for key, cont := range podSpec.InitContainers {
		cont.Env = append(cont.Env, internal.Env("WORKSPACE", robot.Spec.WorkspaceManagerTemplate.WorkspacesPath+"/"+workspace))
		podSpec.InitContainers[key] = cont
	}

	job.Spec.Template.Spec = podSpec

	return job
}

func (cfg *PodConfigInjector) InjectWorkspaceEnvironmentVariable(pod *corev1.Pod, robot robotv1alpha1.Robot, workspace string) *corev1.Pod {

	for key, cont := range pod.Spec.Containers {
		cont.Env = append(cont.Env, internal.Env("WORKSPACE", robot.Spec.WorkspaceManagerTemplate.WorkspacesPath+"/"+workspace))
		pod.Spec.Containers[key] = cont
	}

	for key, cont := range pod.Spec.InitContainers {
		cont.Env = append(cont.Env, internal.Env("WORKSPACE", robot.Spec.WorkspaceManagerTemplate.WorkspacesPath+"/"+workspace))
		pod.Spec.InitContainers[key] = cont
	}

	return pod
}
