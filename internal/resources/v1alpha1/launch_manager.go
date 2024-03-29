package resources

import (
	"path/filepath"
	"strconv"
	"strings"

	"github.com/robolaunch/robot-operator/internal"
	configure "github.com/robolaunch/robot-operator/internal/configure/v1alpha1"
	"github.com/robolaunch/robot-operator/internal/hybrid"
	"github.com/robolaunch/robot-operator/internal/label"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
)

func GetLaunchPod(launchManager *robotv1alpha1.LaunchManager, podNamespacedName *types.NamespacedName, robot robotv1alpha1.Robot, buildManager robotv1alpha1.BuildManager, robotVDI robotv1alpha1.RobotVDI, node corev1.Node) *corev1.Pod {

	cfg := configure.PodConfigInjector{}

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
				configure.GetVolumeConfigMaps(&buildManager),
			},
			RestartPolicy: corev1.RestartPolicyNever,
			NodeSelector:  label.GetTenancyMap(&robot),
		},
	}

	cfg.InjectImagePullPolicy(&launchPod)
	cfg.SchedulePod(&launchPod, launchManager)
	cfg.InjectGenericEnvironmentVariables(&launchPod, robot) // Environment variables
	cfg.InjectLinuxUserAndGroup(&launchPod, robot)           // Linux user and group configuration
	cfg.InjectRuntimeClass(&launchPod, robot, node)
	cfg.InjectVolumeConfiguration(&launchPod, robot)

	if robot.Spec.Type == robotv1alpha1.TypeRobot {
		cfg.InjectRMWImplementationConfiguration(&launchPod, robot) // RMW implementation configuration
		cfg.InjectROSDomainID(&launchPod, robot.Spec.RobotConfig.DomainID)
		cfg.InjectDiscoveryServerConnection(&launchPod, robot.Status.DiscoveryServerStatus.Status.ConnectionInfo) // Discovery server configuration
	}

	if InstanceNeedDisplay(*launchManager, robot) && label.GetTargetRobotVDI(launchManager) != "" {
		// TODO: Add control for validating robot VDI
		cfg.InjectDisplayConfigurationForLaunch(&launchPod, *launchManager, robotVDI) // Display configuration
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

	cfg := configure.ContainerConfigInjector{}

	container := corev1.Container{
		Name:    launchName,
		Image:   robot.Status.Image,
		Command: buildContainerEntrypoint(launch, robot, launch.Entrypoint.DisableSourcingWorkspace),
		Stdin:   true,
		TTY:     true,
		VolumeMounts: []corev1.VolumeMount{
			configure.GetExternalVolumeMount(internal.CUSTOM_SCRIPTS_PATH, configure.GetVolumeConfigMaps(&buildManager)),
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

	cfg.InjectVolumeMountConfiguration(&container, robot, "")
	if launch.Scope.ScopeType == robotv1alpha1.ScopeTypeWorkspace {
		cfg.InjectWorkspaceEnvironmentVariable(&container, robot, launch.Scope.Workspace)
	}

	return container
}

func buildContainerEntrypoint(launch robotv1alpha1.Launch, robot robotv1alpha1.Robot, disableSourcingWs bool) []string {

	sleepTimeInt := 3
	sleepTime := strconv.Itoa(sleepTimeInt)

	var cmdBuilder strings.Builder
	cmdBuilder.WriteString(configure.GetGrantPermissionCmd(robot))
	cmdBuilder.WriteString("echo \"Starting node in " + sleepTime + " seconds...\" && ")
	cmdBuilder.WriteString("sleep " + sleepTime + " && ")

	if launch.Scope.ScopeType == robotv1alpha1.ScopeTypeWorkspace {
		workspace, _ := robot.GetWorkspaceByName(launch.Scope.Workspace)
		cmdBuilder.WriteString("cd $WORKSPACES_PATH/" + launch.Scope.Workspace + " && ")
		if !disableSourcingWs {
			cmdBuilder.WriteString("source " + filepath.Join("/opt", "ros", string(workspace.Distro), "setup.bash") + " && ")
			cmdBuilder.WriteString("source " + filepath.Join("$WORKSPACES_PATH", launch.Scope.Workspace, getWsSubDir(workspace.Distro), "setup.bash") + " && ")
		}
	} else if launch.Scope.ScopeType == robotv1alpha1.ScopeTypePath {
		cmdBuilder.WriteString("cd " + launch.Scope.Path + " && ")
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
