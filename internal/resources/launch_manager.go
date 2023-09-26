package resources

import (
	"path/filepath"
	"strconv"
	"strings"

	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/configure"
	"github.com/robolaunch/robot-operator/internal/hybrid"
	"github.com/robolaunch/robot-operator/internal/label"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
)

func GetLaunchPod(launchManager *robotv1alpha1.LaunchManager, podNamespacedName *types.NamespacedName, robot robotv1alpha1.Robot, buildManager robotv1alpha1.BuildManager, robotVDI robotv1alpha1.RobotVDI, node corev1.Node) *corev1.Pod {

	cfg := configure.ConfigInjector{}

	containers := []corev1.Container{}
	clusterName := label.GetClusterName(&robot)
	for k, l := range launchManager.Spec.Launches {
		if hybrid.ContainsInstance(l.Instances, clusterName) {
			containers = append(containers, getContainer(l, k, robot, buildManager))
		}
	}

	launchPod := corev1.Pod{
		ObjectMeta: metav1.ObjectMeta{
			Name:      podNamespacedName.Name,
			Namespace: podNamespacedName.Namespace,
			Labels:    launchManager.Labels,
		},
		Spec: corev1.PodSpec{
			Containers: containers,
			Volumes: []corev1.Volume{
				configure.GetVolumeVar(&robot),
				configure.GetVolumeUsr(&robot),
				configure.GetVolumeOpt(&robot),
				configure.GetVolumeEtc(&robot),
				configure.GetVolumeWorkspace(&robot),
				configure.GetVolumeConfigMaps(&buildManager),
			},
			RestartPolicy: corev1.RestartPolicyNever,
			NodeSelector:  label.GetTenancyMap(&robot),
		},
	}

	configure.InjectImagePullPolicy(&launchPod)
	configure.SchedulePod(&launchPod, label.GetTenancyMap(launchManager))
	configure.InjectGenericEnvironmentVariables(&launchPod, robot)    // Environment variables
	configure.InjectLinuxUserAndGroup(&launchPod, robot)              // Linux user and group configuration
	configure.InjectRMWImplementationConfiguration(&launchPod, robot) // RMW implementation configuration
	configure.InjectROSDomainID(&launchPod, robot.Spec.RobotConfig.DomainID)
	cfg.InjectDiscoveryServerConnection(&launchPod, robot.Status.DiscoveryServerStatus.Status.ConnectionInfo) // Discovery server configuration
	configure.InjectRuntimeClass(&launchPod, robot, node)

	if InstanceNeedDisplay(*launchManager, robot) && label.GetTargetRobotVDI(launchManager) != "" {
		// TODO: Add control for validating robot VDI
		configure.InjectLaunchPodDisplayConfiguration(&launchPod, *launchManager, robotVDI) // Display configuration
	}

	return &launchPod
}

func InstanceNeedDisplay(launchManager robotv1alpha1.LaunchManager, robot robotv1alpha1.Robot) bool {
	for _, l := range launchManager.Spec.Launches {
		if l.Container.Display && hybrid.ContainsInstance(l.Instances, label.GetClusterName(&robot)) {
			return true
		}
	}
	return false
}

func getContainer(launch robotv1alpha1.Launch, launchName string, robot robotv1alpha1.Robot, buildManager robotv1alpha1.BuildManager) corev1.Container {

	container := corev1.Container{
		Name:    launchName,
		Image:   robot.Status.Image,
		Command: buildContainerEntrypoint(launch, robot, launch.Entrypoint.DisableSourcingWorkspace),
		Stdin:   true,
		TTY:     true,
		VolumeMounts: []corev1.VolumeMount{
			configure.GetVolumeMount("", configure.GetVolumeVar(&robot)),
			configure.GetVolumeMount("", configure.GetVolumeUsr(&robot)),
			configure.GetVolumeMount("", configure.GetVolumeOpt(&robot)),
			configure.GetVolumeMount("", configure.GetVolumeEtc(&robot)),
			configure.GetVolumeMount(robot.Spec.WorkspaceManagerTemplate.WorkspacesPath, configure.GetVolumeWorkspace(&robot)),
			configure.GetVolumeMount(internal.CUSTOM_SCRIPTS_PATH, configure.GetVolumeConfigMaps(&buildManager)),
		},
		Resources: corev1.ResourceRequirements{
			Limits: getResourceLimits(launch.Container.Resources),
		},
		Env:                      launch.Container.Env,
		ImagePullPolicy:          corev1.PullAlways,
		TerminationMessagePolicy: corev1.TerminationMessageReadFile,
		SecurityContext: &corev1.SecurityContext{
			Privileged: &launch.Container.Privileged,
		},
	}

	configure.InjectWorkspaceEnvironmentVariableForContainer(&container, robot, launch.Workspace)

	return container
}

func buildContainerEntrypoint(launch robotv1alpha1.Launch, robot robotv1alpha1.Robot, disableSourcingWs bool) []string {

	sleepTimeInt := 3
	sleepTime := strconv.Itoa(sleepTimeInt)

	workspace, _ := robot.GetWorkspaceByName(launch.Workspace)

	var cmdBuilder strings.Builder
	cmdBuilder.WriteString("echo \"Starting node in " + sleepTime + " seconds...\" && ")
	cmdBuilder.WriteString("sleep " + sleepTime + " && ")
	if !disableSourcingWs {
		cmdBuilder.WriteString("source " + filepath.Join("/opt", "ros", string(workspace.Distro), "setup.bash") + " && ")
		cmdBuilder.WriteString("source " + filepath.Join("$WORKSPACES_PATH", launch.Workspace, getWsSubDir(workspace.Distro), "setup.bash") + " && ")
	}

	switch launch.Entrypoint.Type {
	case robotv1alpha1.LaunchTypeLaunch:
		cmdBuilder.WriteString(generateLaunchCommand(launch, robot))
	case robotv1alpha1.LaunchTypeRun:
		cmdBuilder.WriteString(generateRunCommand(launch, robot))
	case robotv1alpha1.LaunchTypeCustom:
		cmdBuilder.WriteString(generateCustomCommand(launch, robot))
	}

	cmdBuilder.WriteString("$COMMAND")

	return internal.Bash(cmdBuilder.String())
}

func generateLaunchCommand(launch robotv1alpha1.Launch, robot robotv1alpha1.Robot) string {

	robotName := robot.Name

	var parameterBuilder strings.Builder
	for key, val := range launch.Entrypoint.Parameters {
		parameterBuilder.WriteString(key + ":=" + val + " ")
	}

	var cmdBuilder strings.Builder

	cmdBuilder.WriteString("ros2 launch ")
	cmdBuilder.WriteString(launch.Entrypoint.Package + " " + launch.Entrypoint.Launchfile + " ")
	cmdBuilder.WriteString(parameterBuilder.String())

	if launch.Namespacing {
		rosNs := strings.ReplaceAll(robotName, "-", "_")
		cmdBuilder.WriteString(" --namespace " + rosNs)
	}

	return cmdBuilder.String()
}

func generateRunCommand(launch robotv1alpha1.Launch, robot robotv1alpha1.Robot) string {

	robotName := robot.Name

	var parameterBuilder strings.Builder
	if launch.Namespacing {
		rosNs := strings.ReplaceAll(robotName, "-", "_")
		launch.Entrypoint.Parameters["__ns"] = rosNs
	}
	if len(launch.Entrypoint.Parameters) > 0 {
		parameterBuilder.WriteString("--ros-args ")
	}
	for key, val := range launch.Entrypoint.Parameters {
		parameterBuilder.WriteString("-r " + key + ":=" + val + " ")
	}

	var cmdBuilder strings.Builder

	cmdBuilder.WriteString("ros2 run ")
	cmdBuilder.WriteString(launch.Entrypoint.Package + " " + launch.Entrypoint.Executable + " ")
	cmdBuilder.WriteString(parameterBuilder.String())

	return cmdBuilder.String()
}

func generateCustomCommand(launch robotv1alpha1.Launch, robot robotv1alpha1.Robot) string {

	var cmdBuilder strings.Builder
	cmdBuilder.WriteString(launch.Entrypoint.Command)

	return cmdBuilder.String()
}

func getWsSubDir(distro robotv1alpha1.ROSDistro) string {
	if distro == robotv1alpha1.ROSDistroMelodic ||
		distro == robotv1alpha1.ROSDistroNoetic {
		return "devel"
	} else if distro == robotv1alpha1.ROSDistroFoxy ||
		distro == robotv1alpha1.ROSDistroGalactic {
		return "install"
	}
	return "install"
}

func getWorkspaceSourceFilePath(workspacesPath string, wsName string, distro robotv1alpha1.ROSDistro) string {
	return filepath.Join(workspacesPath, wsName, getWsSubDir(distro), "setup.bash")
}

func getLaunchfilePathAbsolute(workspacesPath string, wsName string, repoName string, lfRelativePath string) string {
	return filepath.Join(workspacesPath, wsName, "src", repoName, lfRelativePath)
}
