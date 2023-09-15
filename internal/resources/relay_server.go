package resources

import (
	"fmt"
	"strconv"
	"strings"

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
	RELAY_SERVER_PORT_NAME = "relay-server"
	RELAY_SERVER_PORT      = 7000
)

func getRelayServerSelector(relayserver robotv1alpha1.RelayServer) map[string]string {
	return map[string]string{
		"relay": relayserver.Name,
	}
}

func GetRelayServerPod(relayserver *robotv1alpha1.RelayServer, podNamespacedName *types.NamespacedName) *corev1.Pod {

	var cmdBuilder strings.Builder
	cmdBuilder.WriteString(
		"socat TCP4-LISTEN:" + strconv.Itoa(RELAY_SERVER_PORT) + ",fork,reuseaddr TCP4:" +
			relayserver.Spec.Hostname + "." +
			relayserver.Spec.InstanceName + "." +
			relayserver.Spec.Subdomain + "." +
			relayserver.Spec.RemoteNamespace +
			".svc.clusterset.local:" +
			strconv.Itoa(relayserver.Spec.RemotePort))

	relayServerPod := corev1.Pod{
		ObjectMeta: metav1.ObjectMeta{
			Name:      podNamespacedName.Name,
			Namespace: podNamespacedName.Namespace,
			Labels:    getRelayServerSelector(*relayserver),
		},
		Spec: corev1.PodSpec{
			Containers: []corev1.Container{
				{
					Name:            "relay",
					Image:           "robolaunchio/relay:socat-1.7.3.3-2-focal-0.1.0",
					ImagePullPolicy: corev1.PullIfNotPresent,
					Command:         internal.Bash(cmdBuilder.String()),
					Ports: []corev1.ContainerPort{
						{
							Name:          "http",
							Protocol:      corev1.ProtocolTCP,
							ContainerPort: RELAY_SERVER_PORT,
						},
					},
				},
			},
		},
	}

	configure.InjectImagePullPolicy(&relayServerPod)
	configure.SchedulePod(&relayServerPod, label.GetTenancyMap(relayserver))

	return &relayServerPod
}

func GetRelayServerService(relayserver *robotv1alpha1.RelayServer, svcNamespacedName *types.NamespacedName) *corev1.Service {

	relayServerSvc := corev1.Service{
		ObjectMeta: metav1.ObjectMeta{
			Name:      svcNamespacedName.Name,
			Namespace: svcNamespacedName.Namespace,
		},
		Spec: corev1.ServiceSpec{
			Ports: []corev1.ServicePort{
				{
					Name:     "http",
					Port:     RELAY_SERVER_PORT,
					Protocol: corev1.ProtocolTCP,
					TargetPort: intstr.IntOrString{
						IntVal: RELAY_SERVER_PORT,
					},
				},
			},
			Type:     corev1.ServiceTypeClusterIP,
			Selector: getRelayServerSelector(*relayserver),
		},
	}

	return &relayServerSvc
}

func GetRelayServerIngress(relayserver *robotv1alpha1.RelayServer, ingressNamespacedName *types.NamespacedName) *networkingv1.Ingress {

	tenancy := label.GetTenancy(relayserver)

	rootDNSConfig := relayserver.Spec.RootDNSConfig
	secretName := relayserver.Spec.TLSSecretReference.Name

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
								Path:     robotv1alpha1.GetRelayServerServicePath(*relayserver, "/relay") + "(/|$)(.*)",
								PathType: &pathTypePrefix,
								Backend: networkingv1.IngressBackend{
									Service: &networkingv1.IngressServiceBackend{
										Name: relayserver.GetRelayServerServiceMetadata().Name,
										Port: networkingv1.ServiceBackendPort{
											Number: RELAY_SERVER_PORT,
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
