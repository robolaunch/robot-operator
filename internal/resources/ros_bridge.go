package resources

import (
	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/configure"
	"github.com/robolaunch/robot-operator/internal/label"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/apimachinery/pkg/util/intstr"
)

// TODO: add support for other ROS/2 distros

const (
	ROS_BRIDGE_PORT_NAME  = "bridge-server"
	ROS_BRIDGE_PORT       = 9090
	ROS2_BRIDGE_PORT_NAME = "bridge-server-2"
	ROS2_BRIDGE_PORT      = 9091
)

func getROSBridgeSelector(rosbridge robotv1alpha1.ROSBridge) map[string]string {
	return map[string]string{
		"rosBridge": rosbridge.Name,
	}
}

func GetBridgePod(rosbridge *robotv1alpha1.ROSBridge, podNamespacedName *types.NamespacedName, robot robotv1alpha1.Robot) *corev1.Pod {

	bridgePod := corev1.Pod{
		ObjectMeta: metav1.ObjectMeta{
			Name:      podNamespacedName.Name,
			Namespace: podNamespacedName.Namespace,
			Labels:    getROSBridgeSelector(*rosbridge),
		},
		Spec: corev1.PodSpec{
			Containers: []corev1.Container{},
		},
	}

	if rosbridge.Spec.ROS.Enabled {
		bridgePod.Spec.Containers = append(bridgePod.Spec.Containers, getRoscoreContainer(rosbridge))
		bridgePod.Spec.Containers = append(bridgePod.Spec.Containers, getInterDistroBridgeContainer(rosbridge))
		bridgePod.Spec.Containers = append(bridgePod.Spec.Containers, getROSBridgeContainer(rosbridge))
	}

	if rosbridge.Spec.ROS2.Enabled {
		bridgePod.Spec.Containers = append(bridgePod.Spec.Containers, getROS2BridgeContainer(rosbridge))
	}

	configure.SchedulePod(&bridgePod, label.GetTenancyMap(rosbridge))
	configure.InjectPodDiscoveryServerConnection(&bridgePod, robot.Status.DiscoveryServerStatus.Status.ConnectionInfo)
	configure.InjectRMWImplementationConfiguration(&bridgePod, robot)

	return &bridgePod
}

func GetBridgeService(rosbridge *robotv1alpha1.ROSBridge, svcNamespacedName *types.NamespacedName) *corev1.Service {

	bridgeSvc := corev1.Service{
		ObjectMeta: metav1.ObjectMeta{
			Name:      svcNamespacedName.Name,
			Namespace: svcNamespacedName.Namespace,
		},
		Spec: corev1.ServiceSpec{
			Ports: []corev1.ServicePort{
				{
					Name:     ROS_BRIDGE_PORT_NAME,
					Port:     ROS_BRIDGE_PORT,
					Protocol: corev1.ProtocolTCP,
					TargetPort: intstr.IntOrString{
						IntVal: ROS_BRIDGE_PORT,
					},
				},
				{
					Name:     ROS2_BRIDGE_PORT_NAME,
					Port:     ROS2_BRIDGE_PORT,
					Protocol: corev1.ProtocolTCP,
					TargetPort: intstr.IntOrString{
						IntVal: ROS2_BRIDGE_PORT,
					},
				},
			},
			Type:     corev1.ServiceTypeNodePort,
			Selector: getROSBridgeSelector(*rosbridge),
		},
	}

	return &bridgeSvc
}

func getRoscoreContainer(rosbridge *robotv1alpha1.ROSBridge) corev1.Container {

	rosdistro := rosbridge.Spec.ROS.Distro

	roscoreContainer := corev1.Container{
		Name:    "roscore",
		Image:   rosbridge.Spec.Image,
		Command: internal.Bash("source /opt/ros/" + string(rosdistro) + "/setup.bash && roscore"),
		Env: []corev1.EnvVar{
			internal.Env("ROS_MASTER_URI", "http://127.0.0.1:11311"),
			internal.Env("ROS_HOSTNAME", "127.0.0.1"),
		},
	}

	return roscoreContainer
}

func getInterDistroBridgeContainer(rosbridge *robotv1alpha1.ROSBridge) corev1.Container {

	rosdistro := rosbridge.Spec.ROS.Distro
	ros2distro := rosbridge.Spec.ROS2.Distro

	interDistroBridgeContainer := corev1.Container{
		Name:    "interdistro-bridge",
		Image:   rosbridge.Spec.Image,
		Command: internal.Bash("sleep 3 && source /opt/ros/" + string(rosdistro) + "/setup.bash && source /opt/ros/" + string(ros2distro) + "/setup.bash && ros2 run ros1_bridge dynamic_bridge"),
	}

	// TODO: inject discovery server configuration

	return interDistroBridgeContainer
}

func getROSBridgeContainer(rosbridge *robotv1alpha1.ROSBridge) corev1.Container {

	rosdistro := rosbridge.Spec.ROS.Distro

	rosBridgeContainer := corev1.Container{
		Name:    ROS_BRIDGE_PORT_NAME,
		Image:   rosbridge.Spec.Image,
		Command: internal.Bash("sleep 3 && source /opt/ros/" + string(rosdistro) + "/setup.bash && roslaunch rosbridge_server rosbridge_websocket.launch websocket_external_port:=31044"),
		Env: []corev1.EnvVar{
			internal.Env("ROS_MASTER_URI", "http://127.0.0.1:11311"),
			internal.Env("ROS_HOSTNAME", "127.0.0.1"),
		},
		Ports: []corev1.ContainerPort{
			{
				ContainerPort: ROS_BRIDGE_PORT,
				Name:          ROS_BRIDGE_PORT_NAME,
				Protocol:      corev1.ProtocolTCP,
			},
		},
	}

	return rosBridgeContainer
}

func getROS2BridgeContainer(rosbridge *robotv1alpha1.ROSBridge) corev1.Container {

	ros2distro := rosbridge.Spec.ROS2.Distro

	ros2BridgeContainer := corev1.Container{
		Name:    ROS2_BRIDGE_PORT_NAME,
		Image:   rosbridge.Spec.Image,
		Command: internal.Bash("source /opt/ros/" + string(ros2distro) + "/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=9091 use_compression:=true"),
		Ports: []corev1.ContainerPort{
			{
				ContainerPort: ROS2_BRIDGE_PORT,
				Name:          ROS2_BRIDGE_PORT_NAME,
				Protocol:      corev1.ProtocolTCP,
			},
		},
	}

	// TODO: inject discovery server configuration

	return ros2BridgeContainer
}
