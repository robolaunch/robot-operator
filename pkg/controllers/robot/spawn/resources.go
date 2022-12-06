package spawn

import (
	"path/filepath"
	"strconv"
	"strings"

	batchv1 "k8s.io/api/batch/v1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/resource"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/configure"
	"github.com/robolaunch/robot-operator/internal/label"
	"github.com/robolaunch/robot-operator/internal/node"
)

func GetPersistentVolumeClaim(robot *robotv1alpha1.Robot, pvcNamespacedName *types.NamespacedName) *corev1.PersistentVolumeClaim {

	pvc := corev1.PersistentVolumeClaim{
		ObjectMeta: metav1.ObjectMeta{
			Name:      pvcNamespacedName.Name,
			Namespace: pvcNamespacedName.Namespace,
		},
		Spec: corev1.PersistentVolumeClaimSpec{
			StorageClassName: &robot.Spec.Storage.StorageClassConfig.Name,
			AccessModes: []corev1.PersistentVolumeAccessMode{
				robot.Spec.Storage.StorageClassConfig.AccessMode,
			},
			Resources: corev1.ResourceRequirements{
				Limits: corev1.ResourceList{
					corev1.ResourceName(corev1.ResourceStorage): resource.MustParse(getClaimStorage(pvcNamespacedName, robot.Spec.Storage.Amount)),
				},
				Requests: corev1.ResourceList{
					corev1.ResourceName(corev1.ResourceStorage): resource.MustParse(getClaimStorage(pvcNamespacedName, robot.Spec.Storage.Amount)),
				},
			},
		},
	}

	return &pvc
}

func getClaimStorage(pvc *types.NamespacedName, totalStorage int) string {
	storageInt := 0

	if strings.Contains(pvc.Name, "pvc-var") {
		storageInt = totalStorage / 20
	} else if strings.Contains(pvc.Name, "pvc-opt") {
		storageInt = 3 * totalStorage / 10
	} else if strings.Contains(pvc.Name, "pvc-usr") {
		storageInt = totalStorage * 5 / 10
	} else if strings.Contains(pvc.Name, "pvc-etc") {
		storageInt = totalStorage / 20
	} else if strings.Contains(pvc.Name, "pvc-display") {
		storageInt = 100
	} else if strings.Contains(pvc.Name, "pvc-workspace") {
		storageInt = totalStorage / 10
	} else {
		storageInt = 0
	}
	return strconv.Itoa(storageInt) + "M"

}

func GetDiscoveryServer(robot *robotv1alpha1.Robot, dsNamespacedName *types.NamespacedName) *robotv1alpha1.DiscoveryServer {

	discoveryServer := robotv1alpha1.DiscoveryServer{
		ObjectMeta: metav1.ObjectMeta{
			Name:      dsNamespacedName.Name,
			Namespace: dsNamespacedName.Namespace,
			Labels:    robot.Labels,
		},
		Spec: robot.Spec.DiscoveryServerTemplate,
	}

	return &discoveryServer

}

func GetLoaderJob(robot *robotv1alpha1.Robot, jobNamespacedName *types.NamespacedName, hasGPU bool) *batchv1.Job {

	var copierCmdBuilder strings.Builder
	copierCmdBuilder.WriteString("yes | cp -rf /var /ros/;")
	copierCmdBuilder.WriteString(" yes | cp -rf /usr /ros/;")
	copierCmdBuilder.WriteString(" yes | cp -rf /opt /ros/;")
	copierCmdBuilder.WriteString(" yes | cp -rf /etc /ros/;")
	copierCmdBuilder.WriteString(" echo \"DONE\"")

	readyRobotProp := node.GetReadyRobotProperties(*robot)

	var preparerCmdBuilder strings.Builder
	preparerCmdBuilder.WriteString("mv " + filepath.Join("/etc", "apt", "sources.list.d", "ros2.list") + " temp1")
	preparerCmdBuilder.WriteString(" && mv " + filepath.Join("/etc", "apt", "sources.list") + " temp2")
	preparerCmdBuilder.WriteString(" && apt-get update")
	preparerCmdBuilder.WriteString(" && mv temp1 " + filepath.Join("/etc", "apt", "sources.list.d", "ros2.list"))
	preparerCmdBuilder.WriteString(" && mv temp2 " + filepath.Join("/etc", "apt", "sources.list"))
	preparerCmdBuilder.WriteString(" && apt-get dist-upgrade -y")
	preparerCmdBuilder.WriteString(" && apt-get update")
	if !readyRobotProp.Enabled { // do no run rosdep init if ready robot
		preparerCmdBuilder.WriteString(" && rosdep init")
	}

	copierContainer := corev1.Container{
		Name:    "copier",
		Image:   robot.Status.Image,
		Command: internal.Bash(copierCmdBuilder.String()),
		VolumeMounts: []corev1.VolumeMount{
			configure.GetVolumeMount("/ros/", configure.GetVolumeVar(robot)),
			configure.GetVolumeMount("/ros/", configure.GetVolumeUsr(robot)),
			configure.GetVolumeMount("/ros/", configure.GetVolumeOpt(robot)),
			configure.GetVolumeMount("/ros/", configure.GetVolumeEtc(robot)),
		},
	}

	preparerContainer := corev1.Container{
		Name:    "preparer",
		Image:   "ubuntu:focal",
		Command: internal.Bash(preparerCmdBuilder.String()),
		VolumeMounts: []corev1.VolumeMount{
			configure.GetVolumeMount("", configure.GetVolumeVar(robot)),
			configure.GetVolumeMount("", configure.GetVolumeUsr(robot)),
			configure.GetVolumeMount("", configure.GetVolumeOpt(robot)),
			configure.GetVolumeMount("", configure.GetVolumeEtc(robot)),
		},
	}

	podSpec := &corev1.PodSpec{
		InitContainers: []corev1.Container{
			copierContainer,
		},
		Containers: []corev1.Container{
			preparerContainer,
		},
		Volumes: []corev1.Volume{
			configure.GetVolumeVar(robot),
			configure.GetVolumeUsr(robot),
			configure.GetVolumeOpt(robot),
			configure.GetVolumeEtc(robot),
		},
	}

	if hasGPU {

		var driverInstallerCmdBuilder strings.Builder

		// run /etc/vdi/install-driver.sh
		driverInstallerCmdBuilder.WriteString(filepath.Join("/etc", "vdi", "install-driver.sh"))

		driverInstaller := corev1.Container{
			Name:    "driver-installer",
			Image:   robot.Status.Image,
			Command: internal.Bash(driverInstallerCmdBuilder.String()),
			VolumeMounts: []corev1.VolumeMount{
				configure.GetVolumeMount("", configure.GetVolumeVar(robot)),
				configure.GetVolumeMount("", configure.GetVolumeUsr(robot)),
				configure.GetVolumeMount("", configure.GetVolumeOpt(robot)),
				configure.GetVolumeMount("", configure.GetVolumeEtc(robot)),
			},
		}

		podSpec.InitContainers = append(podSpec.InitContainers, driverInstaller)

	}

	podSpec.RestartPolicy = corev1.RestartPolicyNever
	podSpec.NodeSelector = label.GetTenancyMap(robot)

	job := batchv1.Job{
		ObjectMeta: metav1.ObjectMeta{
			Name:      robot.GetLoaderJobMetadata().Name,
			Namespace: robot.GetLoaderJobMetadata().Namespace,
		},
		Spec: batchv1.JobSpec{
			Template: corev1.PodTemplateSpec{
				Spec: *podSpec,
			},
		},
	}

	return &job
}
