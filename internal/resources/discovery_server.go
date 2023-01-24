package resources

import (
	"fmt"
	"strings"

	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/configure"
	"github.com/robolaunch/robot-operator/internal/label"
	mcsv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/external/apis/mcsv1alpha1/v1alpha1"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
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
			Image: discoveryServer.Spec.Image,
			Args:  discoveryServer.Spec.Args,
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

	configure.SchedulePod(&discoveryServerPod, label.GetTenancyMap(discoveryServer))

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

	superClientConfig := fmt.Sprintf(internal.SUPER_CLIENT_CONFIG, discoveryServer.Status.ConnectionInfo.IP)

	discoveryServerConfigMap := corev1.ConfigMap{
		ObjectMeta: metav1.ObjectMeta{
			Name:      cmNamespacedName.Name,
			Namespace: cmNamespacedName.Namespace,
			Labels: map[string]string{
				"configuredIP": discoveryServer.Status.ConnectionInfo.IP,
			},
		},
		Data: map[string]string{
			"DISCOVERY_SERVER_CONFIG":        superClientConfig,
			"FASTRTPS_DEFAULT_PROFILES_FILE": "/etc/discovery-server/super_client_configuration_file.xml",
			"ROS_DISCOVERY_SERVER":           discoveryServer.Status.ConnectionInfo.IP + ":11811",
		},
	}

	return &discoveryServerConfigMap, nil
}

func GetDiscoveryServerServiceExport(discoveryServer *robotv1alpha1.DiscoveryServer, svcNamespacedName *types.NamespacedName) (*mcsv1alpha1.ServiceExport, error) {

	serviceExport := mcsv1alpha1.ServiceExport{
		ObjectMeta: metav1.ObjectMeta{
			Name:      svcNamespacedName.Name,
			Namespace: svcNamespacedName.Namespace,
		},
	}

	return &serviceExport, nil
}

func GetDiscoveryServerDNS(discoveryServer robotv1alpha1.DiscoveryServer) string {

	dsConfig := discoveryServer.Spec

	cluster := label.GetClusterName(&discoveryServer)

	var serviceDNSBuilder strings.Builder
	if dsConfig.Cluster == cluster {
		if dsConfig.Type == robotv1alpha1.DiscoveryServerInstanceTypeServer {
			// server
			serviceDNSBuilder.WriteString(dsConfig.Hostname + "." + discoveryServer.GetDiscoveryServerServiceMetadata().Name + "." + discoveryServer.Namespace + ".svc." + cluster + ".local")
		} else {
			// client, server is in the same cluster
			serviceDNSBuilder.WriteString(dsConfig.Hostname + "." + dsConfig.Reference.Name + "-" + dsConfig.Subdomain + "." + dsConfig.Reference.Namespace + ".svc." + cluster + ".local")
		}
	} else {
		// client, server is in another cluster
		serviceDNSBuilder.WriteString(dsConfig.Hostname + "." + dsConfig.Cluster + "." + dsConfig.Reference.Name + "-" + dsConfig.Subdomain + "." + dsConfig.Reference.Namespace + ".svc.clusterset.local")
	}

	return serviceDNSBuilder.String()
}
