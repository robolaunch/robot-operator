package resources

import (
	"path/filepath"
	"strings"

	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/configure"
	"github.com/robolaunch/robot-operator/internal/label"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	batchv1 "k8s.io/api/batch/v1"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

func GetConfigMap(buildManager *robotv1alpha1.BuildManager) (*corev1.ConfigMap, error) {

	configScripts := func() map[string]string {
		configScripts := map[string]string{}

		for _, step := range buildManager.Spec.Steps {
			if step.Script != "" {
				configScripts[step.Name] = step.Script
			}
		}

		return configScripts
	}()

	configMap := corev1.ConfigMap{
		ObjectMeta: metav1.ObjectMeta{
			Name:      buildManager.GetConfigMapMetadata().Name,
			Namespace: buildManager.GetConfigMapMetadata().Namespace,
		},
		Data: configScripts,
	}

	return &configMap, nil
}

func GetBuildJob(buildManager *robotv1alpha1.BuildManager, robot *robotv1alpha1.Robot, step robotv1alpha1.Step) *batchv1.Job {

	jobCfg := configure.JobConfigInjector{}
	containerCfg := configure.ContainerConfigInjector{}

	var cmdBuilder strings.Builder
	var cmd []string

	cmdBuilder.WriteString(configure.GetGrantPermissionCmd(*robot))

	if step.Command != "" {
		cmdBuilder.WriteString("cd $WORKSPACES_PATH/" + step.Workspace + " && ")
		cmdBuilder.WriteString(step.Command)
	} else {
		cmdBuilder.WriteString("cd $WORKSPACES_PATH/" + step.Workspace + " && ")
		for _, env := range step.Env {
			cmdBuilder.WriteString(env.Name + "=" + env.Value + " ")
		}
		cmdBuilder.WriteString(filepath.Join(internal.CUSTOM_SCRIPTS_PATH, "scripts", step.Name))
	}

	cmd = internal.Bash(cmdBuilder.String())

	var backoffLimit int32 = 1

	buildContainer := corev1.Container{
		Name:    step.Name,
		Image:   robot.Status.Image,
		Command: cmd,
		VolumeMounts: []corev1.VolumeMount{
			configure.GetExternalVolumeMount(internal.CUSTOM_SCRIPTS_PATH, configure.GetVolumeConfigMaps(buildManager)),
		},
		Env: step.Env,
	}

	containerCfg.InjectVolumeMountConfiguration(&buildContainer, *robot, "")

	podSpec := corev1.PodSpec{
		Containers: []corev1.Container{
			buildContainer,
		},
		Volumes: []corev1.Volume{
			configure.GetVolumeConfigMaps(buildManager),
		},
		RestartPolicy: corev1.RestartPolicyNever,
		NodeSelector:  label.GetTenancyMap(robot),
	}

	job := batchv1.Job{
		ObjectMeta: metav1.ObjectMeta{
			Name:      buildManager.Name + "-" + step.Name,
			Namespace: buildManager.Namespace,
			Labels:    buildManager.Labels,
		},
		Spec: batchv1.JobSpec{
			Template: corev1.PodTemplateSpec{
				Spec: podSpec,
			},
			BackoffLimit: &backoffLimit,
		},
	}

	jobCfg.InjectGenericEnvironmentVariables(&job, *robot)
	jobCfg.InjectWorkspaceEnvironmentVariable(&job, *robot, step.Workspace)
	jobCfg.InjectImagePullPolicy(&job)
	jobCfg.InjectLinuxUserAndGroup(&job, *robot)
	jobCfg.InjectRMWImplementationConfiguration(&job, *robot)
	jobCfg.SchedulePod(&job, robot)
	jobCfg.InjectVolumeConfiguration(&job, *robot)

	return &job
}
