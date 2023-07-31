package resources

import (
	"fmt"

	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/configure"
	"github.com/robolaunch/robot-operator/internal/label"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	networkingv1 "k8s.io/api/networking/v1"
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
	configure.InjectROSDomainID(&bridgePod, robot.Spec.DomainID)

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
			Type:     rosbridge.Spec.ServiceType,
			Selector: getROSBridgeSelector(*rosbridge),
		},
	}

	return &bridgeSvc
}

func GetBridgeIngress(rosBridge *robotv1alpha1.ROSBridge, ingressNamespacedName *types.NamespacedName, robot robotv1alpha1.Robot) *networkingv1.Ingress {

	tenancy := label.GetTenancy(&robot)

	rootDNSConfig := robot.Spec.RootDNSConfig
	secretName := robot.Spec.TLSSecretReference.Name

	annotations := map[string]string{
		internal.INGRESS_AUTH_URL_KEY:                fmt.Sprintf(internal.INGRESS_AUTH_URL_VAL, tenancy.CloudInstanceAlias, rootDNSConfig.Host),
		internal.INGRESS_AUTH_SIGNIN_KEY:             fmt.Sprintf(internal.INGRESS_AUTH_SIGNIN_VAL, tenancy.CloudInstanceAlias, rootDNSConfig.Host),
		internal.INGRESS_AUTH_RESPONSE_HEADERS_KEY:   internal.INGRESS_AUTH_RESPONSE_HEADERS_VAL,
		internal.INGRESS_CONFIGURATION_SNIPPET_KEY:   internal.INGRESS_IDE_CONFIGURATION_SNIPPET_VAL,
		internal.INGRESS_CERT_MANAGER_KEY:            internal.INGRESS_CERT_MANAGER_VAL,
		internal.INGRESS_NGINX_PROXY_BUFFER_SIZE_KEY: internal.INGRESS_NGINX_PROXY_BUFFER_SIZE_VAL,
		internal.INGRESS_NGINX_REWRITE_TARGET_KEY:    internal.INGRESS_NGINX_REWRITE_TARGET_VAL,
		internal.INGRESS_PROXY_READ_TIMEOUT_KEY:      internal.INGRESS_PROXY_READ_TIMEOUT_VAL,
	}

	pathTypePrefix := networkingv1.PathTypePrefix
	ingressClassNameNginx := "nginx"

	ingressSpec := networkingv1.IngressSpec{
		TLS: []networkingv1.IngressTLS{
			{
				Hosts: []string{
					tenancy.CloudInstanceAlias + "." + rootDNSConfig.Host,
				},
				SecretName: secretName,
			},
		},
		Rules: []networkingv1.IngressRule{
			{
				Host: tenancy.CloudInstanceAlias + "." + rootDNSConfig.Host,
				IngressRuleValue: networkingv1.IngressRuleValue{
					HTTP: &networkingv1.HTTPIngressRuleValue{
						Paths: []networkingv1.HTTPIngressPath{
							{
								Path:     robotv1alpha1.GetRobotServicePath(robot, "/bridge") + "(/|$)(.*)",
								PathType: &pathTypePrefix,
								Backend: networkingv1.IngressBackend{
									Service: &networkingv1.IngressServiceBackend{
										Name: rosBridge.GetBridgeServiceMetadata().Name,
										Port: networkingv1.ServiceBackendPort{
											Number: ROS2_BRIDGE_PORT,
										},
									},
								},
							},
						},
					},
				},
			},
		},
		IngressClassName: &ingressClassNameNginx,
	}

	ingress := &networkingv1.Ingress{
		ObjectMeta: metav1.ObjectMeta{
			Name:        ingressNamespacedName.Name,
			Namespace:   ingressNamespacedName.Namespace,
			Annotations: annotations,
		},
		Spec: ingressSpec,
	}

	return ingress
}

func getRoscoreContainer(rosbridge *robotv1alpha1.ROSBridge) corev1.Container {

	rosdistro := rosbridge.Spec.ROS.Distro

	roscoreContainer := corev1.Container{
		Name:    "roscore",
		Image:   "robolaunchio/foxy-noetic-bridge:v0.0.3",
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
		Image:   "robolaunchio/foxy-noetic-bridge:v0.0.3",
		Command: internal.Bash("sleep 3 && source /opt/ros/" + string(rosdistro) + "/setup.bash && source /opt/ros/" + string(ros2distro) + "/setup.bash && ros2 run ros1_bridge dynamic_bridge"),
	}

	// TODO: inject discovery server configuration

	return interDistroBridgeContainer
}

func getROSBridgeContainer(rosbridge *robotv1alpha1.ROSBridge) corev1.Container {

	rosdistro := rosbridge.Spec.ROS.Distro

	rosBridgeContainer := corev1.Container{
		Name:    ROS_BRIDGE_PORT_NAME,
		Image:   "robolaunchio/foxy-noetic-bridge:v0.0.3",
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
		Image:   "robolaunchio/foxy-noetic-bridge:v0.0.3",
		Command: internal.Bash("source /opt/ros/" + string(ros2distro) + "/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=9091 use_compression:=true ssl:=true"),
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
