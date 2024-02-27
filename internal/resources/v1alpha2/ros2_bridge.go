package v1alpha2_resources

import (
	"reflect"

	corev1 "k8s.io/api/core/v1"
	networkingv1 "k8s.io/api/networking/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"

	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/configure"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
)

const (
	ROS_BRIDGE_PORT_NAME  = "bridge-server"
	ROS_BRIDGE_PORT       = 9090
	ROS2_BRIDGE_PORT_NAME = "bridge-server-2"
	ROS2_BRIDGE_PORT      = 9091
)

func getROSBridgeSelector(rosbridge robotv1alpha2.ROS2Bridge) map[string]string {
	return map[string]string{
		"ros2Bridge": rosbridge.Name,
	}
}

func GetROS2BridgePod(ros2bridge *robotv1alpha2.ROS2Bridge, podNamespacedName *types.NamespacedName, image string, discoveryServer robotv1alpha1.DiscoveryServer) *corev1.Pod {
	cfg := configure.PodConfigInjector{}

	ros2BridgeContainer := corev1.Container{
		Name:    ROS2_BRIDGE_PORT_NAME,
		Image:   image,
		Command: internal.Bash("source /opt/ros/" + string(ros2bridge.Spec.Distro) + "/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=9091 use_compression:=true ssl:=true"),
		Ports: []corev1.ContainerPort{
			{
				ContainerPort: ROS2_BRIDGE_PORT,
				Name:          ROS2_BRIDGE_PORT_NAME,
				Protocol:      corev1.ProtocolTCP,
			},
		},
	}

	bridgePod := corev1.Pod{
		ObjectMeta: metav1.ObjectMeta{
			Name:      podNamespacedName.Name,
			Namespace: podNamespacedName.Namespace,
			Labels:    getROSBridgeSelector(*ros2bridge),
		},
		Spec: corev1.PodSpec{
			Containers: []corev1.Container{
				ros2BridgeContainer,
			},
		},
	}

	cfg.InjectImagePullPolicy(&bridgePod)
	cfg.SchedulePod(&bridgePod, ros2bridge)

	if reflect.DeepEqual(ros2bridge.Status.DiscoveryServerReference, corev1.ObjectReference{}) {
		cfg.InjectDiscoveryServerConnection(&bridgePod, discoveryServer.Status.ConnectionInfo)
		cfg.InjectROSDomainID(&bridgePod, discoveryServer.Spec.DomainID)
		// cfg.InjectRMWImplementationConfiguration(&bridgePod, robot)
	}

	return &bridgePod
}

func GetROS2BridgeService(rosbridge *robotv1alpha2.ROS2Bridge, podNamespacedName *types.NamespacedName) *corev1.Service {
	return &corev1.Service{}
}

func GetROS2BridgeIngress(rosBridge *robotv1alpha2.ROS2Bridge, ingressNamespacedName *types.NamespacedName) *networkingv1.Ingress {
	return &networkingv1.Ingress{}
}
