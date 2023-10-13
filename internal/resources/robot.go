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

func GetPersistentVolumeClaim(robot *robotv1alpha1.Robot, pDir robotv1alpha1.PersistentDirectory) *corev1.PersistentVolumeClaim {

	pvc := corev1.PersistentVolumeClaim{
		ObjectMeta: metav1.ObjectMeta{
			Name:      pDir.Status.Reference.Name,
			Namespace: pDir.Status.Reference.Namespace,
		},
		Spec: corev1.PersistentVolumeClaimSpec{
			StorageClassName: &robot.Spec.Storage.StorageClassConfig.Name,
			AccessModes: []corev1.PersistentVolumeAccessMode{
				robot.Spec.Storage.StorageClassConfig.AccessMode,
			},
			Resources: corev1.ResourceRequirements{
				Limits: corev1.ResourceList{
					corev1.ResourceName(corev1.ResourceStorage): resource.MustParse(getClaimStorage(pDir, robot.Spec.Storage.Amount, *robot)),
				},
				Requests: corev1.ResourceList{
					corev1.ResourceName(corev1.ResourceStorage): resource.MustParse(getClaimStorage(pDir, robot.Spec.Storage.Amount, *robot)),
				},
			},
		},
	}

	return &pvc
}

func getClaimStorage(pDir robotv1alpha1.PersistentDirectory, totalStorage int, robot robotv1alpha1.Robot) string {
	storageInt := 0

	if pDir.Path == "/var" {
		storageInt = totalStorage / 20
	} else if pDir.Path == "/opt" {
		storageInt = 3 * totalStorage / 10
	} else if pDir.Path == "/usr" {
		storageInt = totalStorage * 5 / 10
	} else if pDir.Path == "/etc" {
		storageInt = totalStorage / 20
	} else if pDir.Path == "/tmp/.X11-unix" {
		storageInt = 100
	} else if pDir.Path == robot.Spec.WorkspaceManagerTemplate.WorkspacesPath {
		storageInt = totalStorage / 10
	} else {
		storageInt = 200
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
		Spec: robot.Spec.RobotConfig.DiscoveryServerTemplate,
	}

	return &discoveryServer

}

func GetLoaderJob(robot *robotv1alpha1.Robot, jobNamespacedName *types.NamespacedName, hasGPU bool) *batchv1.Job {

	if robot.Spec.Type == robotv1alpha1.TypeRobot {
		return GetLoaderJobForRobot(robot, jobNamespacedName, hasGPU)
	} else if robot.Spec.Type == robotv1alpha1.TypeEnvironment {
		return GetLoaderJobForEnvironment(robot, jobNamespacedName, hasGPU)
	}

	return nil
}

func GetLoaderJobForRobot(robot *robotv1alpha1.Robot, jobNamespacedName *types.NamespacedName, hasGPU bool) *batchv1.Job {

	cfg := configure.JobConfigInjector{}
	containerCfg := configure.ContainerConfigInjector{}

	var copierCmdBuilder strings.Builder
	copierCmdBuilder.WriteString("yes | cp -rf /var /ros/;")
	copierCmdBuilder.WriteString(" yes | cp -rf /usr /ros/;")
	copierCmdBuilder.WriteString(" yes | cp -rf /opt /ros/;")
	copierCmdBuilder.WriteString(" yes | cp -rf /etc /ros/;")
	copierCmdBuilder.WriteString(" echo \"DONE\"")

	var uidGetterCmdBuilder strings.Builder
	uidGetterCmdBuilder.WriteString("id -u robolaunch")

	readyRobotProp := node.GetReadyRobotProperties(*robot)

	closerStr := "&&"

	if _, ok := robot.Labels[internal.OFFLINE_LABEL_KEY]; ok {
		closerStr = ";"
	}

	var preparerCmdBuilder strings.Builder
	preparerCmdBuilder.WriteString("mv " + filepath.Join("/etc", "apt", "sources.list.d", "ros2.list") + " temp1 ")
	preparerCmdBuilder.WriteString(closerStr + " mv " + filepath.Join("/etc", "apt", "sources.list") + " temp2 ")
	preparerCmdBuilder.WriteString(closerStr + " apt-get update")
	preparerCmdBuilder.WriteString(closerStr + " mv temp1 " + filepath.Join("/etc", "apt", "sources.list.d", "ros2.list "))
	preparerCmdBuilder.WriteString(closerStr + " mv temp2 " + filepath.Join("/etc", "apt", "sources.list") + " ")
	preparerCmdBuilder.WriteString(closerStr + " apt-get update ")
	preparerCmdBuilder.WriteString(closerStr + " chown root:root /usr/bin/sudo ")
	preparerCmdBuilder.WriteString(closerStr + " chmod 4755 /usr/bin/sudo ")

	// grant permission for directories
	// directories and files can be specified in $PATH format: eg. "/opt:/etc/files:/usr/lib"
	var paths []string
	if val, ok := robot.Spec.AdditionalConfigs[internal.GRANT_PERMISSION_KEY]; ok && val.ConfigType == robotv1alpha1.AdditionalConfigTypeOperator {
		paths = strings.Split(val.Value, ":")
	}

	for _, v := range paths {
		preparerCmdBuilder.WriteString(closerStr + " setfacl -R -m u:robolaunch:rwx " + v + " ")
	}

	if !readyRobotProp.Enabled { // do no run rosdep init if ready robot
		preparerCmdBuilder.WriteString(closerStr + " rosdep init")
	}

	copierContainer := corev1.Container{
		Name:            "copier",
		Image:           robot.Status.Image,
		Command:         internal.Bash(copierCmdBuilder.String()),
		ImagePullPolicy: corev1.PullAlways,
	}

	containerCfg.InjectVolumeMountConfiguration(&copierContainer, *robot, "/ros")

	uidGetterContainer := corev1.Container{
		Name:    "uid-getter",
		Image:   robot.Status.Image,
		Command: internal.Bash(uidGetterCmdBuilder.String()),
	}

	containerCfg.InjectVolumeMountConfiguration(&uidGetterContainer, *robot, "")

	preparerContainer := corev1.Container{
		Name:    "preparer",
		Image:   "ubuntu:focal",
		Command: internal.Bash(preparerCmdBuilder.String()),
	}

	containerCfg.InjectVolumeMountConfiguration(&preparerContainer, *robot, "")

	podSpec := &corev1.PodSpec{
		InitContainers: []corev1.Container{
			copierContainer,
			uidGetterContainer,
		},
		Containers: []corev1.Container{
			preparerContainer,
			// clonerContainer,
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
		}

		containerCfg.InjectVolumeMountConfiguration(&driverInstaller, *robot, "")

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

	cfg.InjectGenericEnvironmentVariables(&job, *robot)
	cfg.InjectImagePullPolicy(&job)
	cfg.InjectVolumeConfiguration(&job, *robot)

	return &job
}

func GetLoaderJobForEnvironment(robot *robotv1alpha1.Robot, jobNamespacedName *types.NamespacedName, hasGPU bool) *batchv1.Job {

	cfg := configure.JobConfigInjector{}
	containerCfg := configure.ContainerConfigInjector{}

	var copierCmdBuilder strings.Builder
	copierCmdBuilder.WriteString("yes | cp -rf /var /environment/;")
	copierCmdBuilder.WriteString(" yes | cp -rf /usr /environment/;")
	copierCmdBuilder.WriteString(" yes | cp -rf /opt /environment/;")
	copierCmdBuilder.WriteString(" yes | cp -rf /etc /environment/;")
	copierCmdBuilder.WriteString(" echo \"DONE\"")

	var uidGetterCmdBuilder strings.Builder
	uidGetterCmdBuilder.WriteString("id -u robolaunch")

	closerStr := "&&"

	if _, ok := robot.Labels[internal.OFFLINE_LABEL_KEY]; ok {
		closerStr = ";"
	}

	var preparerCmdBuilder strings.Builder
	preparerCmdBuilder.WriteString("mv " + filepath.Join("/etc", "apt", "sources.list") + " temp ")
	preparerCmdBuilder.WriteString(closerStr + " apt-get update ")
	preparerCmdBuilder.WriteString(closerStr + " mv temp " + filepath.Join("/etc", "apt", "sources.list") + " ")
	preparerCmdBuilder.WriteString(closerStr + " apt-get update ")
	preparerCmdBuilder.WriteString(closerStr + " chown root:root /usr/bin/sudo ")
	preparerCmdBuilder.WriteString(closerStr + " chmod 4755 /usr/bin/sudo ")

	// grant permission for directories
	// directories and files can be specified in $PATH format: eg. "/opt:/etc/files:/usr/lib"
	var paths []string
	if val, ok := robot.Spec.AdditionalConfigs[internal.GRANT_PERMISSION_KEY]; ok && val.ConfigType == robotv1alpha1.AdditionalConfigTypeOperator {
		paths = strings.Split(val.Value, ":")
	}

	for _, v := range paths {
		preparerCmdBuilder.WriteString(closerStr + " setfacl -R -m u:robolaunch:rwx " + v + " ")
	}

	copierContainer := corev1.Container{
		Name:            "copier",
		Image:           robot.Status.Image,
		Command:         internal.Bash(copierCmdBuilder.String()),
		ImagePullPolicy: corev1.PullAlways,
	}

	containerCfg.InjectVolumeMountConfiguration(&copierContainer, *robot, "/environment")

	uidGetterContainer := corev1.Container{
		Name:    "uid-getter",
		Image:   robot.Status.Image,
		Command: internal.Bash(uidGetterCmdBuilder.String()),
	}

	containerCfg.InjectVolumeMountConfiguration(&uidGetterContainer, *robot, "")

	preparerContainer := corev1.Container{
		Name:    "preparer",
		Image:   "ubuntu:focal",
		Command: internal.Bash(preparerCmdBuilder.String()),
	}

	containerCfg.InjectVolumeMountConfiguration(&preparerContainer, *robot, "")

	podSpec := &corev1.PodSpec{
		InitContainers: []corev1.Container{
			copierContainer,
			uidGetterContainer,
		},
		Containers: []corev1.Container{
			preparerContainer,
			// clonerContainer,
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
		}

		containerCfg.InjectVolumeMountConfiguration(&driverInstaller, *robot, "")

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

	cfg.InjectGenericEnvironmentVariables(&job, *robot)
	cfg.InjectImagePullPolicy(&job)
	cfg.SchedulePod(&job, robot)
	cfg.InjectVolumeConfiguration(&job, *robot)

	return &job
}

func GetROSBridge(robot *robotv1alpha1.Robot, bridgeNamespacedName *types.NamespacedName) *robotv1alpha1.ROSBridge {

	rosBridge := robotv1alpha1.ROSBridge{
		ObjectMeta: metav1.ObjectMeta{
			Name:      bridgeNamespacedName.Name,
			Namespace: bridgeNamespacedName.Namespace,
			Labels:    robot.Labels,
		},
		Spec: robot.Spec.RobotConfig.ROSBridgeTemplate,
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

func GetCloneCommand(robot robotv1alpha1.Robot, workspaces []robotv1alpha1.Workspace, wsKey int) string {

	var cmdBuilder strings.Builder
	closerStr := "&&"
	if _, ok := robot.Labels[internal.OFFLINE_LABEL_KEY]; ok {
		closerStr = ";"
	}
	for key, repo := range workspaces[wsKey].Repositories {
		cmdBuilder.WriteString("git clone --recursive " + repo.URL + " -b " + repo.Branch + " " + key + " " + closerStr)
	}
	return cmdBuilder.String()
}
