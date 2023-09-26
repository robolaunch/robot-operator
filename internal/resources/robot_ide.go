package resources

import (
	"fmt"
	"strconv"
	"strings"

	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/configure"
	"github.com/robolaunch/robot-operator/internal/label"
	mcsv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/external/apis/mcsv1alpha1/v1alpha1"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	networkingv1 "k8s.io/api/networking/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/apimachinery/pkg/util/intstr"
)

const (
	ROBOT_IDE_PORT_NAME = "code-server"
	ROBOT_IDE_PORT      = 9000
)

func getRobotIDESelector(robotIDE robotv1alpha1.RobotIDE) map[string]string {
	return map[string]string{
		"robotIDE": robotIDE.Name,
	}
}

func GetRobotIDEPod(robotIDE *robotv1alpha1.RobotIDE, podNamespacedName *types.NamespacedName, robot robotv1alpha1.Robot, robotVDI robotv1alpha1.RobotVDI, node corev1.Node) *corev1.Pod {

	cfg := configure.PodConfigInjector{}

	var cmdBuilder strings.Builder
	cmdBuilder.WriteString("code-server " + robot.Spec.WorkspaceManagerTemplate.WorkspacesPath + " --bind-addr 0.0.0.0:$CODE_SERVER_PORT --auth none")

	labels := getRobotIDESelector(*robotIDE)
	for k, v := range robotIDE.Labels {
		labels[k] = v
	}

	pod := corev1.Pod{
		ObjectMeta: metav1.ObjectMeta{
			Name:      podNamespacedName.Name,
			Namespace: podNamespacedName.Namespace,
			Labels:    labels,
		},
		Spec: corev1.PodSpec{
			Containers: []corev1.Container{
				{
					Name:    "code-server",
					Image:   robot.Status.Image,
					Command: internal.Bash(cmdBuilder.String()),
					Env: []corev1.EnvVar{
						internal.Env("CODE_SERVER_PORT", strconv.Itoa(ROBOT_IDE_PORT)),
						internal.Env("ROBOT_NAMESPACE", robot.Namespace),
						internal.Env("ROBOT_NAME", robot.Name),
						internal.Env("TERM", "xterm-256color"),
					},
					VolumeMounts: []corev1.VolumeMount{
						configure.GetVolumeMount("", configure.GetVolumeVar(&robot)),
						configure.GetVolumeMount("", configure.GetVolumeUsr(&robot)),
						configure.GetVolumeMount("", configure.GetVolumeOpt(&robot)),
						configure.GetVolumeMount("", configure.GetVolumeEtc(&robot)),
						configure.GetVolumeMount(robot.Spec.WorkspaceManagerTemplate.WorkspacesPath, configure.GetVolumeWorkspace(&robot)),
					},
					Ports: []corev1.ContainerPort{
						{
							Name:          ROBOT_IDE_PORT_NAME,
							ContainerPort: ROBOT_IDE_PORT,
						},
					},
					Resources: corev1.ResourceRequirements{
						Limits: getResourceLimits(robotIDE.Spec.Resources),
					},
					SecurityContext: &corev1.SecurityContext{
						Privileged: &robotIDE.Spec.Privileged,
					},
				},
			},
			Volumes: []corev1.Volume{
				configure.GetVolumeVar(&robot),
				configure.GetVolumeUsr(&robot),
				configure.GetVolumeOpt(&robot),
				configure.GetVolumeEtc(&robot),
				configure.GetVolumeWorkspace(&robot),
			},
			RestartPolicy: corev1.RestartPolicyNever,
		},
	}

	configure.InjectImagePullPolicy(&pod)
	configure.SchedulePod(&pod, label.GetTenancyMap(robotIDE))
	configure.InjectGenericEnvironmentVariables(&pod, robot)
	configure.InjectLinuxUserAndGroup(&pod, robot)
	configure.InjectRuntimeClass(&pod, robot, node)
	if robotIDE.Spec.Display && label.GetTargetRobotVDI(robotIDE) != "" {
		// TODO: Add control for validating robot VDI
		cfg.InjectDisplayConfiguration(&pod, robotVDI)
	}

	if label.GetInstanceType(&robot) == label.InstanceTypePhysicalInstance {
		// apply ONLY if the resource is on physical instance
		configure.InjectRemoteConfigurationsForPod(&pod, *robotIDE)
	}

	if robot.Spec.Type == robotv1alpha1.TypeRobot {
		configure.InjectGenericRobotEnvironmentVariables(&pod, robot)
		configure.InjectRMWImplementationConfiguration(&pod, robot)
		configure.InjectROSDomainID(&pod, robot.Spec.RobotConfig.DomainID)
		cfg.InjectDiscoveryServerConnection(&pod, robot.Status.DiscoveryServerStatus.Status.ConnectionInfo)
	}

	return &pod
}

func GetRobotIDEService(robotIDE *robotv1alpha1.RobotIDE, svcNamespacedName *types.NamespacedName) *corev1.Service {

	serviceSpec := corev1.ServiceSpec{
		Type:     robotIDE.Spec.ServiceType,
		Selector: getRobotIDESelector(*robotIDE),
		Ports: []corev1.ServicePort{
			{
				Port: ROBOT_IDE_PORT,
				TargetPort: intstr.IntOrString{
					IntVal: ROBOT_IDE_PORT,
				},
				Protocol: corev1.ProtocolTCP,
				Name:     ROBOT_IDE_PORT_NAME,
			},
		},
	}

	service := corev1.Service{
		ObjectMeta: metav1.ObjectMeta{
			Name:      svcNamespacedName.Name,
			Namespace: svcNamespacedName.Namespace,
		},
		Spec: serviceSpec,
	}

	if label.GetInstanceType(robotIDE) == label.InstanceTypePhysicalInstance {
		// apply ONLY if the resource is on physical instance
		configure.InjectRemoteConfigurationsForService(&service)
	}

	return &service
}

func GetRobotIDEIngress(robotIDE *robotv1alpha1.RobotIDE, ingressNamespacedName *types.NamespacedName, robot robotv1alpha1.Robot) *networkingv1.Ingress {

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
								Path:     robotv1alpha1.GetRobotServicePath(robot, "/ide") + "(/|$)(.*)",
								PathType: &pathTypePrefix,
								Backend: networkingv1.IngressBackend{
									Service: &networkingv1.IngressServiceBackend{
										Name: robotIDE.GetRobotIDEServiceMetadata().Name,
										Port: networkingv1.ServiceBackendPort{
											Number: ROBOT_IDE_PORT,
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

func GetRobotIDEServiceExport(robotIDE *robotv1alpha1.RobotIDE, svcExportNamespacedName *types.NamespacedName) *mcsv1alpha1.ServiceExport {

	serviceExport := mcsv1alpha1.ServiceExport{
		ObjectMeta: metav1.ObjectMeta{
			Name:      svcExportNamespacedName.Name,
			Namespace: svcExportNamespacedName.Namespace,
		},
	}

	return &serviceExport
}
