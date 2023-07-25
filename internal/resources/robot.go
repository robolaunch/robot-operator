package resources

import (
	"path/filepath"
	"strconv"
	"strings"

	batchv1 "k8s.io/api/batch/v1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/resource"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"

	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/configure"
	"github.com/robolaunch/robot-operator/internal/label"
	"github.com/robolaunch/robot-operator/internal/node"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
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
	preparerCmdBuilder.WriteString(" && chown root:root /usr/bin/sudo")
	preparerCmdBuilder.WriteString(" && chmod 4755 /usr/bin/sudo")
	if !readyRobotProp.Enabled { // do no run rosdep init if ready robot
		preparerCmdBuilder.WriteString(" && rosdep init")
	}

	var clonerCmdBuilder strings.Builder
	for wsKey, ws := range robot.Spec.WorkspaceManagerTemplate.Workspaces {

		var cmdBuilder strings.Builder
		cmdBuilder.WriteString("mkdir -p " + filepath.Join(robot.Spec.WorkspaceManagerTemplate.WorkspacesPath, ws.Name, "src") + " && ")
		cmdBuilder.WriteString("cd " + filepath.Join(robot.Spec.WorkspaceManagerTemplate.WorkspacesPath, ws.Name, "src") + " && ")
		cmdBuilder.WriteString(GetCloneCommand(robot.Spec.WorkspaceManagerTemplate.Workspaces, wsKey))
		clonerCmdBuilder.WriteString(cmdBuilder.String())

	}

	clonerCmdBuilder.WriteString("echo \"DONE\"")

	copierContainer := corev1.Container{
		Name:            "copier",
		Image:           robot.Status.Image,
		Command:         internal.Bash(copierCmdBuilder.String()),
		ImagePullPolicy: corev1.PullAlways,
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

	// clonerContainer := corev1.Container{
	// 	Name:    "cloner",
	// 	Image:   "ubuntu:focal",
	// 	Command: internal.Bash(clonerCmdBuilder.String()),
	// 	VolumeMounts: []corev1.VolumeMount{
	// 		configure.GetVolumeMount("", configure.GetVolumeVar(robot)),
	// 		configure.GetVolumeMount("", configure.GetVolumeUsr(robot)),
	// 		configure.GetVolumeMount("", configure.GetVolumeOpt(robot)),
	// 		configure.GetVolumeMount("", configure.GetVolumeEtc(robot)),
	// 		configure.GetVolumeMount(robot.Spec.WorkspacesPath, configure.GetVolumeWorkspace(robot)),
	// 	},
	// }

	podSpec := &corev1.PodSpec{
		InitContainers: []corev1.Container{
			copierContainer,
		},
		Containers: []corev1.Container{
			preparerContainer,
			// clonerContainer,
		},
		Volumes: []corev1.Volume{
			configure.GetVolumeVar(robot),
			configure.GetVolumeUsr(robot),
			configure.GetVolumeOpt(robot),
			configure.GetVolumeEtc(robot),
			configure.GetVolumeWorkspace(robot),
		},
	}

	if hasGPU {

		var driverInstallerCmdBuilder strings.Builder

		// run /etc/vdi/install-driver.sh
		driverInstallerCmdBuilder.WriteString(filepath.Join("/etc", "vdi", "install-driver.sh"))

		driverInstaller := corev1.Container{
			Name:            "driver-installer",
			Image:           robot.Status.Image,
			Command:         internal.Bash(driverInstallerCmdBuilder.String()),
			ImagePullPolicy: corev1.PullAlways,
			Env: []corev1.EnvVar{
				internal.Env("NVIDIA_DRIVER_VERSION", "agnostic"),
				internal.Env("RESOLUTION", robot.Spec.RobotDevSuiteTemplate.RobotVDITemplate.Resolution),
			},
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

func GetROSBridge(robot *robotv1alpha1.Robot, bridgeNamespacedName *types.NamespacedName) *robotv1alpha1.ROSBridge {

	rosBridge := robotv1alpha1.ROSBridge{
		ObjectMeta: metav1.ObjectMeta{
			Name:      bridgeNamespacedName.Name,
			Namespace: bridgeNamespacedName.Namespace,
			Labels:    robot.Labels,
		},
		Spec: robot.Spec.ROSBridgeTemplate,
	}

	return &rosBridge

}

func GetRobotDevSuite(robot *robotv1alpha1.Robot, rdsNamespacedName *types.NamespacedName) *robotv1alpha1.RobotDevSuite {

	labels := robot.Labels
	labels[internal.TARGET_ROBOT_LABEL_KEY] = robot.Name
	labels[internal.ROBOT_DEV_SUITE_OWNED] = "true"

	robotDevSuite := robotv1alpha1.RobotDevSuite{
		ObjectMeta: metav1.ObjectMeta{
			Name:      rdsNamespacedName.Name,
			Namespace: rdsNamespacedName.Namespace,
			Labels:    robot.Labels,
		},
		Spec: robot.Spec.RobotDevSuiteTemplate,
	}

	return &robotDevSuite

}

func GetWorkspaceManager(robot *robotv1alpha1.Robot, wsmNamespacedName *types.NamespacedName) *robotv1alpha1.WorkspaceManager {

	labels := robot.Labels
	labels[internal.TARGET_ROBOT_LABEL_KEY] = robot.Name

	workspaceManager := robotv1alpha1.WorkspaceManager{
		ObjectMeta: metav1.ObjectMeta{
			Name:      wsmNamespacedName.Name,
			Namespace: wsmNamespacedName.Namespace,
			Labels:    robot.Labels,
		},
		Spec: robot.Spec.WorkspaceManagerTemplate,
	}

	return &workspaceManager

}

func GetBuildManager(robot *robotv1alpha1.Robot, bmNamespacedName *types.NamespacedName) *robotv1alpha1.BuildManager {

	labels := robot.Labels
	labels[internal.TARGET_ROBOT_LABEL_KEY] = robot.Name

	buildManager := robotv1alpha1.BuildManager{
		ObjectMeta: metav1.ObjectMeta{
			Name:      bmNamespacedName.Name,
			Namespace: bmNamespacedName.Namespace,
			Labels:    robot.Labels,
		},
		Spec: robot.Spec.BuildManagerTemplate,
	}

	return &buildManager
}

func GetLaunchManager(robot *robotv1alpha1.Robot, lmNamespacedName *types.NamespacedName, key int) *robotv1alpha1.LaunchManager {

	labels := robot.Labels
	labels[internal.TARGET_ROBOT_LABEL_KEY] = robot.Name
	labels[internal.TARGET_VDI_LABEL_KEY] = robot.GetRobotDevSuiteMetadata().Name + internal.ROBOT_VDI_POSTFIX

	launchManager := robotv1alpha1.LaunchManager{
		ObjectMeta: metav1.ObjectMeta{
			Name:      lmNamespacedName.Name,
			Namespace: lmNamespacedName.Namespace,
			Labels:    robot.Labels,
		},
		Spec: robot.Spec.LaunchManagerTemplates[key],
	}

	return &launchManager
}

func GetCloneCommand(workspaces []robotv1alpha1.Workspace, wsKey int) string {

	var cmdBuilder strings.Builder
	for key, repo := range workspaces[wsKey].Repositories {
		cmdBuilder.WriteString("git clone --recursive " + repo.URL + " -b " + repo.Branch + " " + key + " &&")
	}
	return cmdBuilder.String()
}
