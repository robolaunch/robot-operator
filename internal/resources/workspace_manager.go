package resources

import (
	"path/filepath"
	"strconv"
	"strings"

	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/configure"
	"github.com/robolaunch/robot-operator/internal/label"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	batchv1 "k8s.io/api/batch/v1"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
)

func GetClonerJob(workspaceManager *robotv1alpha1.WorkspaceManager, jobNamespacedName *types.NamespacedName, robot *robotv1alpha1.Robot) *batchv1.Job {

	cfg := configure.JobConfigInjector{}

	var clonerCmdBuilder strings.Builder
	for wsKey, ws := range workspaceManager.Spec.Workspaces {

		var cmdBuilder strings.Builder
		cmdBuilder.WriteString("mkdir -p " + filepath.Join(workspaceManager.Spec.WorkspacesPath, ws.Name, "src") + " && ")
		cmdBuilder.WriteString("cd " + filepath.Join(workspaceManager.Spec.WorkspacesPath, ws.Name, "src") + " && ")
		cmdBuilder.WriteString(GetCloneCommand(*robot, workspaceManager.Spec.Workspaces, wsKey))
		clonerCmdBuilder.WriteString(cmdBuilder.String())

	}

	clonerCmdBuilder.WriteString("echo \"DONE\"")

	clonerContainer := corev1.Container{
		Name:    "cloner",
		Image:   "ubuntu:focal",
		Command: internal.Bash(clonerCmdBuilder.String()),
		VolumeMounts: []corev1.VolumeMount{
			configure.GetVolumeMount("", configure.GetVolumeVar(robot)),
			configure.GetVolumeMount("", configure.GetVolumeUsr(robot)),
			configure.GetVolumeMount("", configure.GetVolumeOpt(robot)),
			configure.GetVolumeMount("", configure.GetVolumeEtc(robot)),
			configure.GetVolumeMount(workspaceManager.Spec.WorkspacesPath, configure.GetVolumeWorkspace(robot)),
		},
	}

	podSpec := &corev1.PodSpec{
		Containers: []corev1.Container{
			clonerContainer,
		},
		Volumes: []corev1.Volume{
			configure.GetVolumeVar(robot),
			configure.GetVolumeUsr(robot),
			configure.GetVolumeOpt(robot),
			configure.GetVolumeEtc(robot),
			configure.GetVolumeWorkspace(robot),
		},
	}

	podSpec.RestartPolicy = corev1.RestartPolicyNever
	podSpec.NodeSelector = label.GetTenancyMap(robot)

	job := batchv1.Job{
		ObjectMeta: metav1.ObjectMeta{
			Name:      workspaceManager.GetClonerJobMetadata().Name,
			Namespace: workspaceManager.GetClonerJobMetadata().Namespace,
		},
		Spec: batchv1.JobSpec{
			Template: corev1.PodTemplateSpec{
				Spec: *podSpec,
			},
		},
	}

	cfg.InjectImagePullPolicy(podSpec)
	cfg.InjectLinuxUserAndGroup(&job, *robot)

	return &job
}

func GetCleanupJob(workspaceManager *robotv1alpha1.WorkspaceManager, jobNamespacedName *types.NamespacedName, robot *robotv1alpha1.Robot) *batchv1.Job {

	cfg := configure.JobConfigInjector{}

	var cmdBuilder strings.Builder
	cmdBuilder.WriteString("cd " + workspaceManager.Spec.WorkspacesPath + " && ")
	cmdBuilder.WriteString("GLOBIGNORE=old &&")
	cmdBuilder.WriteString("mkdir -p old/backup-" + strconv.Itoa(workspaceManager.Status.Version) + " &&")
	cmdBuilder.WriteString("mv * old/backup-" + strconv.Itoa(workspaceManager.Status.Version) + " || true &&")
	cmdBuilder.WriteString("rm -rf *")

	cleanupContainer := corev1.Container{
		Name:    "cleanup",
		Image:   "ubuntu:focal",
		Command: internal.Bash(cmdBuilder.String()),
		VolumeMounts: []corev1.VolumeMount{
			configure.GetVolumeMount("", configure.GetVolumeVar(robot)),
			configure.GetVolumeMount("", configure.GetVolumeUsr(robot)),
			configure.GetVolumeMount("", configure.GetVolumeOpt(robot)),
			configure.GetVolumeMount("", configure.GetVolumeEtc(robot)),
			configure.GetVolumeMount(workspaceManager.Spec.WorkspacesPath, configure.GetVolumeWorkspace(robot)),
		},
	}

	podSpec := &corev1.PodSpec{
		Containers: []corev1.Container{
			cleanupContainer,
		},
		Volumes: []corev1.Volume{
			configure.GetVolumeVar(robot),
			configure.GetVolumeUsr(robot),
			configure.GetVolumeOpt(robot),
			configure.GetVolumeEtc(robot),
			configure.GetVolumeWorkspace(robot),
		},
	}

	podSpec.RestartPolicy = corev1.RestartPolicyNever
	podSpec.NodeSelector = label.GetTenancyMap(robot)

	job := batchv1.Job{
		ObjectMeta: metav1.ObjectMeta{
			Name:      workspaceManager.GetCleanupJobMetadata().Name,
			Namespace: workspaceManager.GetCleanupJobMetadata().Namespace,
		},
		Spec: batchv1.JobSpec{
			Template: corev1.PodTemplateSpec{
				Spec: *podSpec,
			},
		},
	}

	cfg.InjectLinuxUserAndGroup(&job, *robot)

	return &job
}
