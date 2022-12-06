package spawn

import (
	"fmt"
	"net"
	"strings"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	"github.com/robolaunch/robot-operator/internal"
	robotErr "github.com/robolaunch/robot-operator/internal/error"
	"github.com/robolaunch/robot-operator/internal/label"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/apimachinery/pkg/util/intstr"
)

var discoveryServerPortName = "discovery"
var discoveryServerPortNumber = 11811

func getDiscoveryServerSelector(discoveryServer robotv1alpha1.DiscoveryServer) map[string]string {
	return map[string]string{
		"discoveryServer": discoveryServer.Name,
	}
}

func GetDiscoveryServerPod(discoveryServer *robotv1alpha1.DiscoveryServer, podNamespacedName *types.NamespacedName) *corev1.Pod {

	containers := []corev1.Container{
		{
			Name:  "discovery-server",
			Image: "deeyi2000/ubuntu-fastdds-suite:v2.4.0",
			Args: []string{
				"fast-discovery-server-1.0.0",
				"-i",
				"0",
			},
			Ports: []corev1.ContainerPort{
				{
					Name:          discoveryServerPortName,
					ContainerPort: int32(discoveryServerPortNumber),
				},
			},
		},
	}

	discoveryServerPod := corev1.Pod{
		ObjectMeta: metav1.ObjectMeta{
			Name:      podNamespacedName.Name,
			Namespace: podNamespacedName.Namespace,
			Labels:    getDiscoveryServerSelector(*discoveryServer),
		},
		Spec: corev1.PodSpec{
			Containers:    containers,
			RestartPolicy: corev1.RestartPolicyNever,
			Hostname:      discoveryServer.Spec.Hostname,
			Subdomain:     discoveryServer.GetDiscoveryServerServiceMetadata().Name,
		},
	}

	return &discoveryServerPod
}

func GetDiscoveryServerService(discoveryServer *robotv1alpha1.DiscoveryServer, svcNamespacedName *types.NamespacedName) *corev1.Service {

	discoveryServerService := corev1.Service{
		ObjectMeta: metav1.ObjectMeta{
			Name:      svcNamespacedName.Name,
			Namespace: svcNamespacedName.Namespace,
		},
		Spec: corev1.ServiceSpec{
			ClusterIP: "None",
			Ports: []corev1.ServicePort{
				{
					Port: int32(discoveryServerPortNumber),
					TargetPort: intstr.IntOrString{
						IntVal: int32(discoveryServerPortNumber),
					},
					Protocol: corev1.ProtocolTCP,
					Name:     discoveryServerPortName,
				},
			},
			Selector: getDiscoveryServerSelector(*discoveryServer),
		},
	}

	return &discoveryServerService
}

func GetDiscoveryServerConfigMap(discoveryServer *robotv1alpha1.DiscoveryServer, cmNamespacedName *types.NamespacedName) (*corev1.ConfigMap, error) {

	dnsName := GetDiscoveryServerDNS(*discoveryServer)

	ips, err := net.LookupIP(dnsName)
	if err != nil {
		return nil, &robotErr.CannotResolveDiscoveryServerError{
			ResourceKind:      discoveryServer.Kind,
			ResourceName:      discoveryServer.Name,
			ResourceNamespace: discoveryServer.Namespace,
		}
	} else if len(ips) == 0 {
		return nil, &robotErr.CannotResolveDiscoveryServerError{
			ResourceKind:      discoveryServer.Kind,
			ResourceName:      discoveryServer.Name,
			ResourceNamespace: discoveryServer.Namespace,
		}
	}

	superClientConfig := fmt.Sprintf(internal.SUPER_CLIENT_CONFIG, ips[0])

	discoveryServerConfigMap := corev1.ConfigMap{
		ObjectMeta: metav1.ObjectMeta{
			Name:      cmNamespacedName.Name,
			Namespace: cmNamespacedName.Namespace,
		},
		Data: map[string]string{"super_client_configuration_file.xml": superClientConfig},
	}

	return &discoveryServerConfigMap, nil
}

func GetDiscoveryServerDNS(discoveryServer robotv1alpha1.DiscoveryServer) string {

	dsConfig := discoveryServer.Spec
	cluster := label.GetClusterName(&discoveryServer)

	var serviceDNSBuilder strings.Builder
	if dsConfig.Attached {
		serviceDNSBuilder.WriteString(dsConfig.Hostname + "." + discoveryServer.GetDiscoveryServerServiceMetadata().Name + "." + discoveryServer.Namespace + ".svc." + cluster + ".local")
	} else {
		serviceDNSBuilder.WriteString(dsConfig.Hostname + "." + dsConfig.Cluster + "." + discoveryServer.GetDiscoveryServerServiceMetadata().Name + "." + discoveryServer.Namespace + ".svc.clusterset.local")
	}

	return serviceDNSBuilder.String()
}
