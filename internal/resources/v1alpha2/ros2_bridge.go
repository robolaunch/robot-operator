package v1alpha2_resources

import (
	"fmt"
	"reflect"

	corev1 "k8s.io/api/core/v1"
	networkingv1 "k8s.io/api/networking/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/apimachinery/pkg/util/intstr"

	"github.com/robolaunch/robot-operator/internal"
	configure "github.com/robolaunch/robot-operator/internal/configure/v1alpha1"
	"github.com/robolaunch/robot-operator/internal/label"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
)

const (
	ROS2_BRIDGE_PORT_NAME = "bridge-server"
	ROS2_BRIDGE_PORT      = 9090
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

	if !reflect.DeepEqual(ros2bridge.Status.ConnectionInfo, robotv1alpha1.ConnectionInfo{}) {
		cfg.InjectDiscoveryServerConnection(&bridgePod, ros2bridge.Status.ConnectionInfo)
		cfg.InjectROSDomainID(&bridgePod, discoveryServer.Spec.DomainID)
		// cfg.InjectRMWImplementationConfiguration(&bridgePod, robot)
	}

	return &bridgePod
}

func GetROS2BridgeService(rosbridge *robotv1alpha2.ROS2Bridge, svcNamespacedName *types.NamespacedName) *corev1.Service {
	bridgeSvc := corev1.Service{
		ObjectMeta: metav1.ObjectMeta{
			Name:      svcNamespacedName.Name,
			Namespace: svcNamespacedName.Namespace,
		},
		Spec: corev1.ServiceSpec{
			Ports: []corev1.ServicePort{
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

func GetROS2BridgeIngress(ros2Bridge *robotv1alpha2.ROS2Bridge, ingressNamespacedName *types.NamespacedName) *networkingv1.Ingress {

	tenancy := label.GetTenancy(ros2Bridge)
	platformMeta := label.GetPlatformMeta(ros2Bridge)

	annotations := map[string]string{
		internal.INGRESS_AUTH_URL_KEY:                fmt.Sprintf(internal.INGRESS_AUTH_URL_VAL, tenancy.Team, platformMeta.Domain),
		internal.INGRESS_AUTH_SIGNIN_KEY:             fmt.Sprintf(internal.INGRESS_AUTH_SIGNIN_VAL, tenancy.Team, platformMeta.Domain),
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
					tenancy.Team + "." + platformMeta.Domain,
				},
				SecretName: ros2Bridge.Spec.TLSSecretName,
			},
		},
		Rules: []networkingv1.IngressRule{
			{
				Host: tenancy.Team + "." + platformMeta.Domain,
				IngressRuleValue: networkingv1.IngressRuleValue{
					HTTP: &networkingv1.HTTPIngressRuleValue{
						Paths: []networkingv1.HTTPIngressPath{
							{
								Path:     robotv1alpha1.GetServicePath(ros2Bridge, "/bridge") + "(/|$)(.*)",
								PathType: &pathTypePrefix,
								Backend: networkingv1.IngressBackend{
									Service: &networkingv1.IngressServiceBackend{
										Name: ros2Bridge.GetROS2BridgeServiceMetadata().Name,
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
