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

	containers := []corev1.Container{}
	clusterName := label.GetClusterName(&robot)
	for k, l := range launchManager.Spec.Launch {
		if hybrid.ContainsInstance(l.Instances, clusterName) {
			cont := corev1.Container{}
			switch l.Type {
			case robotv1alpha1.LaunchTypeLaunch:
				cont = getLaunchContainer(l, k, robot, buildManager)
			case robotv1alpha1.LaunchTypeRun:
				cont = getRunContainer(l, k, robot, buildManager)

			}
			containers = append(containers, cont)
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

	configure.SchedulePod(&launchPod, label.GetTenancyMap(launchManager))
	configure.InjectGenericEnvironmentVariables(&launchPod, robot)                                                     // Environment variables
	configure.InjectRMWImplementationConfiguration(&launchPod, robot)                                                  // RMW implementation configuration
	configure.InjectPodDiscoveryServerConnection(&launchPod, robot.Status.DiscoveryServerStatus.Status.ConnectionInfo) // Discovery server configuration
	configure.InjectRuntimeClass(&launchPod, robot, node)
	if launchManager.Spec.Display && label.GetTargetRobotVDI(launchManager) != "" {
		// TODO: Add control for validating robot VDI
		configure.InjectPodDisplayConfiguration(&launchPod, robotVDI) // Display configuration
	}

	return &launchPod
}

func getLaunchContainer(launch robotv1alpha1.Launch, launchName string, robot robotv1alpha1.Robot, buildManager robotv1alpha1.BuildManager) corev1.Container {

	sleepTimeInt := 3
	sleepTime := strconv.Itoa(sleepTimeInt)

	workspace, _ := robot.GetWorkspaceByName(launch.Workspace)

	var cmdBuilder strings.Builder
	cmdBuilder.WriteString("echo \"Starting node in " + sleepTime + " seconds...\" && ")
	cmdBuilder.WriteString("sleep " + sleepTime + " && ")
	cmdBuilder.WriteString("source " + filepath.Join("/opt", "ros", string(workspace.Distro), "setup.bash") + " && ")
	cmdBuilder.WriteString("source " + filepath.Join("$WORKSPACES_PATH", launch.Workspace, getWsSubDir(workspace.Distro), "setup.bash") + " && ")
	cmdBuilder.WriteString("$PRELAUNCH; ")
	cmdBuilder.WriteString("$COMMAND")

	container := corev1.Container{
		Name:    launchName,
		Image:   robot.Status.Image,
		Command: internal.Bash(cmdBuilder.String()),
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
			Limits: getResourceLimits(launch.Resources),
		},
		Env: []corev1.EnvVar{
			GeneratePrelaunchCommandAsEnv(launch.Prelaunch, robot),
			GenerateLaunchCommandAsEnv(launch, robot),
		},
		ImagePullPolicy:          corev1.PullAlways,
		TerminationMessagePolicy: corev1.TerminationMessageReadFile,
		SecurityContext: &corev1.SecurityContext{
			Privileged: &launch.Privileged,
		},
	}

	return container
}

func getRunContainer(launch robotv1alpha1.Launch, launchName string, robot robotv1alpha1.Robot, buildManager robotv1alpha1.BuildManager) corev1.Container {

	sleepTimeInt := 3
	sleepTime := strconv.Itoa(sleepTimeInt)

	workspace, _ := robot.GetWorkspaceByName(launch.Workspace)

	var cmdBuilder strings.Builder
	cmdBuilder.WriteString("echo \"Starting node in " + sleepTime + " seconds...\" && ")
	cmdBuilder.WriteString("sleep " + sleepTime + " && ")
	cmdBuilder.WriteString("source " + filepath.Join("/opt", "ros", string(workspace.Distro), "setup.bash") + " && ")
	cmdBuilder.WriteString("source " + filepath.Join("$WORKSPACES_PATH", launch.Workspace, getWsSubDir(workspace.Distro), "setup.bash") + " && ")
	cmdBuilder.WriteString("$PRELAUNCH; ")
	cmdBuilder.WriteString("$COMMAND")

	container := corev1.Container{
		Name:    launchName,
		Image:   robot.Status.Image,
		Command: internal.Bash(cmdBuilder.String()),
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
			Limits: getResourceLimits(launch.Resources),
		},
		Env: []corev1.EnvVar{
			GeneratePrelaunchCommandAsEnv(launch.Prelaunch, robot),
			GenerateRunCommandAsEnv(launch, robot),
		},
		ImagePullPolicy:          corev1.PullAlways,
		TerminationMessagePolicy: corev1.TerminationMessageReadFile,
		SecurityContext: &corev1.SecurityContext{
			Privileged: &launch.Privileged,
		},
	}

	return container
}

func GetWorkspaceSourceFilePath(workspacesPath string, wsName string, distro robotv1alpha1.ROSDistro) string {
	return filepath.Join(workspacesPath, wsName, getWsSubDir(distro), "setup.bash")
}

func GetLaunchfilePathAbsolute(workspacesPath string, wsName string, repoName string, lfRelativePath string) string {
	return filepath.Join(workspacesPath, wsName, "src", repoName, lfRelativePath)
}

func GenerateLaunchCommandAsEnv(launch robotv1alpha1.Launch, robot robotv1alpha1.Robot) corev1.EnvVar {

	robotName := robot.Name
	commandKey := "COMMAND"

	var parameterBuilder strings.Builder
	for key, val := range launch.Parameters {
		parameterBuilder.WriteString(key + ":=" + val + " ")
	}

	var cmdBuilder strings.Builder

	cmdBuilder.WriteString("ros2 launch ")
	cmdBuilder.WriteString(launch.Package + " " + launch.Launchfile + " ")
	cmdBuilder.WriteString(parameterBuilder.String())

	if launch.Namespacing {
		rosNs := strings.ReplaceAll(robotName, "-", "_")
		cmdBuilder.WriteString(" --namespace " + rosNs)
	}

	return internal.Env(commandKey, cmdBuilder.String())
}

func GenerateRunCommandAsEnv(launch robotv1alpha1.Launch, robot robotv1alpha1.Robot) corev1.EnvVar {

	robotName := robot.Name

	commandKey := "COMMAND"

	var parameterBuilder strings.Builder
	if launch.Namespacing {
		rosNs := strings.ReplaceAll(robotName, "-", "_")
		launch.Parameters["__ns"] = rosNs
	}
	if len(launch.Parameters) > 0 {
		parameterBuilder.WriteString("--ros-args ")
	}
	for key, val := range launch.Parameters {
		parameterBuilder.WriteString("-r " + key + ":=" + val + " ")
	}

	var cmdBuilder strings.Builder

	cmdBuilder.WriteString("ros2 run ")
	cmdBuilder.WriteString(launch.Package + " " + launch.Executable + " ")
	cmdBuilder.WriteString(parameterBuilder.String())

	return internal.Env(commandKey, cmdBuilder.String())
}

func GeneratePrelaunchCommandAsEnv(prelaunch robotv1alpha1.Prelaunch, robot robotv1alpha1.Robot) corev1.EnvVar {

	commandKey := "PRELAUNCH"
	command := prelaunch.Command

	if command == "" {
		command = "sleep 1"
	}

	return internal.Env(commandKey, command)
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
