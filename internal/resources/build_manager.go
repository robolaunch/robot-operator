package resources

import (
	"path/filepath"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/configure"
	"github.com/robolaunch/robot-operator/internal/label"
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

	var cmd []string
	if step.Command != "" {
		cmd = internal.Bash(step.Command)

	} else {
		cmd = internal.Bash(filepath.Join(internal.CUSTOM_SCRIPTS_PATH, "scripts", step.Name))
	}

	var backoffLimit int32 = 1

	job := batchv1.Job{
		ObjectMeta: metav1.ObjectMeta{
			Name:      buildManager.Name + "-" + step.Name,
			Namespace: buildManager.Namespace,
		},
		Spec: batchv1.JobSpec{
			Template: corev1.PodTemplateSpec{
				Spec: corev1.PodSpec{
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
								configure.GetVolumeMount(robotSpec.WorkspacesPath, configure.GetVolumeWorkspace(robot)),
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
				},
			},
			BackoffLimit: &backoffLimit,
		},
	}

	return &job
}
