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

	robotSpec := robot.Spec

	var cmdBuilder strings.Builder
	var cmd []string
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

	podSpec := corev1.PodSpec{
		Containers: []corev1.Container{
			{
				Name:    step.Name,
				Image:   robot.Status.Image,
				Command: cmd,
				VolumeMounts: []corev1.VolumeMount{
					configure.GetVolumeMount("", configure.GetVolumeVar(robot)),
					configure.GetVolumeMount("", configure.GetVolumeUsr(robot)),
					configure.GetVolumeMount("", configure.GetVolumeOpt(robot)),
					configure.GetVolumeMount("", configure.GetVolumeEtc(robot)),
					configure.GetVolumeMount(robotSpec.WorkspaceManagerTemplate.WorkspacesPath, configure.GetVolumeWorkspace(robot)),
					configure.GetVolumeMount(internal.CUSTOM_SCRIPTS_PATH, configure.GetVolumeConfigMaps(buildManager)),
				},
				Env: step.Env,
			},
		},
		Volumes: []corev1.Volume{
			configure.GetVolumeVar(robot),
			configure.GetVolumeUsr(robot),
			configure.GetVolumeOpt(robot),
			configure.GetVolumeEtc(robot),
			configure.GetVolumeWorkspace(robot),
			configure.GetVolumeConfigMaps(buildManager),
		},
		RestartPolicy: corev1.RestartPolicyNever,
		NodeSelector:  label.GetTenancyMap(robot),
	}

	configure.InjectGenericEnvironmentVariablesForPodSpec(&podSpec, *robot)
	configure.InjectRMWImplementationConfigurationForPodSpec(&podSpec, *robot)

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

	return &job
}
