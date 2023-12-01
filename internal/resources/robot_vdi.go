package resources

import (
	"fmt"
	"path/filepath"
	"strconv"
	"strings"

	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/configure"
	"github.com/robolaunch/robot-operator/internal/label"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	networkingv1 "k8s.io/api/networking/v1"
	"k8s.io/apimachinery/pkg/api/resource"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/apimachinery/pkg/util/intstr"
)

const (
	ROBOT_VDI_PORT_NAME = "http"
	ROBOT_VDI_PORT      = 8055
)

func getRobotVDISelector(robotVDI robotv1alpha1.RobotVDI) map[string]string {
	return map[string]string{
		"robotVDI": robotVDI.Name,
	}
}

func GetRobotVDIPVC(robotVDI *robotv1alpha1.RobotVDI, pvcNamespacedName *types.NamespacedName, robot robotv1alpha1.Robot) *corev1.PersistentVolumeClaim {

	pvc := corev1.PersistentVolumeClaim{
		ObjectMeta: metav1.ObjectMeta{
			Name:      pvcNamespacedName.Name,
			Namespace: pvcNamespacedName.Namespace,
		},
		Spec: corev1.PersistentVolumeClaimSpec{
			StorageClassName: &robot.Spec.Storage.StorageClassConfig.Name,
			AccessModes: []corev1.PersistentVolumeAccessMode{
				robot.Spec.Storage.StorageClassConfig.AccessMode,
			},
			Resources: corev1.ResourceRequirements{
				Limits: corev1.ResourceList{
					corev1.ResourceName(corev1.ResourceStorage): resource.MustParse("100"),
				},
				Requests: corev1.ResourceList{
					corev1.ResourceName(corev1.ResourceStorage): resource.MustParse("100"),
				},
			},
		},
	}

	return &pvc
}

func GetRobotVDIPod(robotVDI *robotv1alpha1.RobotVDI, podNamespacedName *types.NamespacedName, robot robotv1alpha1.Robot, node corev1.Node) *corev1.Pod {

	podCfg := configure.PodConfigInjector{}
	containerCfg := configure.ContainerConfigInjector{}

	// add tcp port
	ports := []corev1.ContainerPort{
		{
			Name:          ROBOT_VDI_PORT_NAME,
			ContainerPort: ROBOT_VDI_PORT,
			Protocol:      corev1.ProtocolTCP,
		},
		{
			Name:          internal.FILE_BROWSER_PORT_NAME,
			ContainerPort: internal.FILE_BROWSER_PORT,
			Protocol:      corev1.ProtocolTCP,
		},
	}

	// add udp ports
	rangeLimits := strings.Split(robotVDI.Spec.WebRTCPortRange, "-")
	rangeStart, _ := strconv.Atoi(rangeLimits[0])
	rangeEnd, _ := strconv.Atoi(rangeLimits[1])

	counter := 0
	for i := rangeStart; i <= rangeEnd; i++ {
		counter++
		ports = append(ports, corev1.ContainerPort{
			Name:          "webrtc-" + strconv.Itoa(counter),
			ContainerPort: int32(i),
			Protocol:      corev1.ProtocolUDP,
		})
	}

	icelite := "true"
	if robotVDI.Spec.NAT1TO1 != "" {
		icelite = "false"
	}

	var cmdBuilder strings.Builder
	cmdBuilder.WriteString(configure.GetGrantPermissionCmd(robot))
	cmdBuilder.WriteString(filepath.Join("/etc", "vdi", "generate-xorg.sh") + " && ")
	cmdBuilder.WriteString("supervisord -c " + filepath.Join("/etc", "robolaunch", "services", "vdi.conf"))

	labels := getRobotVDISelector(*robotVDI)
	for k, v := range robotVDI.Labels {
		labels[k] = v
	}

	vdiContainer := corev1.Container{
		Name:    "vdi",
		Image:   robot.Status.Image,
		Command: internal.Bash(cmdBuilder.String()),
		Env: []corev1.EnvVar{
			internal.Env("VIDEO_PORT", "DFP"),
			internal.Env("NEKO_BIND", ":8055"),
			internal.Env("NEKO_EPR", robotVDI.Spec.WebRTCPortRange),
			internal.Env("NEKO_ICELITE", icelite),
			internal.Env("NEKO_NAT1TO1", robotVDI.Spec.NAT1TO1),
			internal.Env("RESOLUTION", robotVDI.Spec.Resolution),
			internal.Env("FILE_BROWSER_PORT", strconv.Itoa(internal.FILE_BROWSER_PORT)),
		},
		Stdin: true,
		TTY:   true,
		Ports: ports,
		VolumeMounts: []corev1.VolumeMount{
			configure.GetExternalVolumeMount("/dev/shm", configure.GetVolumeDshm()),
			configure.GetExternalVolumeMount("/cache", configure.GetVolumeXglCache()),
		},
		Resources: corev1.ResourceRequirements{
			Limits: getResourceLimits(robotVDI.Spec.Resources),
		},
		ImagePullPolicy:          corev1.PullAlways,
		TerminationMessagePolicy: corev1.TerminationMessageReadFile,
		SecurityContext: &corev1.SecurityContext{
			Privileged: &robotVDI.Spec.Privileged,
		},
	}

	containerCfg.InjectVolumeMountConfiguration(&vdiContainer, robot, "")
	// add custom ports defined by user
	if ports, ok := robot.Spec.AdditionalConfigs[internal.VDI_CUSTOM_PORT_RANGE_KEY]; ok {
		containerCfg.InjectCustomPortConfiguration(&vdiContainer, ports)
	}

	vdiPod := &corev1.Pod{
		ObjectMeta: metav1.ObjectMeta{
			Name:      podNamespacedName.Name,
			Namespace: podNamespacedName.Namespace,
			Labels:    labels,
		},
		Spec: corev1.PodSpec{
			// HostNetwork: robotVDI.Spec.Privileged,
			Containers: []corev1.Container{
				vdiContainer,
			},
			Volumes: []corev1.Volume{
				configure.GetVolumeDshm(),
				configure.GetVolumeXglCache(),
			},
			RestartPolicy: corev1.RestartPolicyNever,
		},
	}

	podCfg.InjectImagePullPolicy(vdiPod)
	podCfg.SchedulePod(vdiPod, robotVDI)
	podCfg.InjectGenericEnvironmentVariables(vdiPod, robot)
	podCfg.InjectDisplayConfiguration(vdiPod, *robotVDI)
	podCfg.InjectRuntimeClass(vdiPod, robot, node)
	podCfg.InjectVolumeConfiguration(vdiPod, robot)

	if !robotVDI.Spec.DisableNVENC {
		podCfg.InjectEncodingOption(vdiPod, robot)
	}

	if robot.Spec.Type == robotv1alpha1.TypeRobot {
		podCfg.InjectGenericRobotEnvironmentVariables(vdiPod, robot)
		podCfg.InjectRMWImplementationConfiguration(vdiPod, robot)
		podCfg.InjectROSDomainID(vdiPod, robot.Spec.RobotConfig.DomainID)
		podCfg.InjectDiscoveryServerConnection(vdiPod, robot.Status.DiscoveryServerStatus.Status.ConnectionInfo)
	}

	return vdiPod
}

func GetRobotVDIServiceTCP(robotVDI *robotv1alpha1.RobotVDI, svcNamespacedName *types.NamespacedName) *corev1.Service {

	ports := []corev1.ServicePort{
		{
			Port: ROBOT_VDI_PORT,
			TargetPort: intstr.IntOrString{
				IntVal: int32(ROBOT_VDI_PORT),
			},
			Protocol: corev1.ProtocolTCP,
			Name:     ROBOT_VDI_PORT_NAME,
		},
		{
			Port: internal.FILE_BROWSER_PORT,
			TargetPort: intstr.IntOrString{
				IntVal: int32(internal.FILE_BROWSER_PORT),
			},
			Protocol: corev1.ProtocolTCP,
			Name:     internal.FILE_BROWSER_PORT_NAME,
		},
	}

	serviceSpec := corev1.ServiceSpec{
		Type:     robotVDI.Spec.ServiceType,
		Selector: getRobotVDISelector(*robotVDI),
		Ports:    ports,
	}

	service := corev1.Service{
		ObjectMeta: metav1.ObjectMeta{
			Name:      svcNamespacedName.Name,
			Namespace: svcNamespacedName.Namespace,
		},
		Spec: serviceSpec,
	}

	return &service
}

func GetRobotVDIServiceUDP(robotVDI *robotv1alpha1.RobotVDI, svcNamespacedName *types.NamespacedName) *corev1.Service {

	ports := []corev1.ServicePort{}

	// add udp ports
	rangeLimits := strings.Split(robotVDI.Spec.WebRTCPortRange, "-")
	rangeStart, _ := strconv.Atoi(rangeLimits[0])
	rangeEnd, _ := strconv.Atoi(rangeLimits[1])

	counter := 0
	for i := rangeStart; i <= rangeEnd; i++ {
		counter++
		ports = append(ports, corev1.ServicePort{
			Name: "webrtc-" + strconv.Itoa(counter),
			Port: int32(i),
			TargetPort: intstr.IntOrString{
				IntVal: int32(i),
			},
			NodePort: int32(i),
			Protocol: corev1.ProtocolUDP,
		})
	}

	serviceSpec := corev1.ServiceSpec{
		Type:                  corev1.ServiceTypeNodePort,
		ExternalTrafficPolicy: corev1.ServiceExternalTrafficPolicyTypeLocal,
		Selector:              getRobotVDISelector(*robotVDI),
		Ports:                 ports,
	}

	service := corev1.Service{
		ObjectMeta: metav1.ObjectMeta{
			Name:      svcNamespacedName.Name,
			Namespace: svcNamespacedName.Namespace,
		},
		Spec: serviceSpec,
	}

	return &service
}

func GetRobotVDIIngress(robotVDI *robotv1alpha1.RobotVDI, ingressNamespacedName *types.NamespacedName, robot robotv1alpha1.Robot) *networkingv1.Ingress {

	tenancy := label.GetTenancy(&robot)

	rootDNSConfig := robot.Spec.RootDNSConfig
	secretName := robot.Spec.TLSSecretReference.Name

	annotations := map[string]string{
		internal.INGRESS_AUTH_URL_KEY:                   fmt.Sprintf(internal.INGRESS_AUTH_URL_VAL, tenancy.CloudInstanceAlias, rootDNSConfig.Host),
		internal.INGRESS_AUTH_SIGNIN_KEY:                fmt.Sprintf(internal.INGRESS_AUTH_SIGNIN_VAL, tenancy.CloudInstanceAlias, rootDNSConfig.Host),
		internal.INGRESS_AUTH_RESPONSE_HEADERS_KEY:      internal.INGRESS_AUTH_RESPONSE_HEADERS_VAL,
		internal.INGRESS_CONFIGURATION_SNIPPET_KEY:      internal.INGRESS_VDI_CONFIGURATION_SNIPPET_VAL,
		internal.INGRESS_CERT_MANAGER_KEY:               internal.INGRESS_CERT_MANAGER_VAL,
		internal.INGRESS_NGINX_PROXY_BUFFER_SIZE_KEY:    internal.INGRESS_NGINX_PROXY_BUFFER_SIZE_VAL,
		internal.INGRESS_NGINX_PROXY_BUFFERS_NUMBER_KEY: internal.INGRESS_VDI_NGINX_PROXY_BUFFERS_NUMBER_VAL,
		internal.INGRESS_NGINX_REWRITE_TARGET_KEY:       internal.INGRESS_NGINX_REWRITE_TARGET_VAL,
		internal.INGRESS_PROXY_READ_TIMEOUT_KEY:         internal.INGRESS_PROXY_READ_TIMEOUT_VAL,
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
								Path:     robotv1alpha1.GetRobotServicePath(robot, "/vdi") + "(/|$)(.*)",
								PathType: &pathTypePrefix,
								Backend: networkingv1.IngressBackend{
									Service: &networkingv1.IngressServiceBackend{
										Name: robotVDI.GetRobotVDIServiceTCPMetadata().Name,
										Port: networkingv1.ServiceBackendPort{
											Number: int32(ROBOT_VDI_PORT),
										},
									},
								},
							},
							{
								Path:     robotv1alpha1.GetRobotServicePath(robot, "/file-browser/vdi") + "(/|$)(.*)",
								PathType: &pathTypePrefix,
								Backend: networkingv1.IngressBackend{
									Service: &networkingv1.IngressServiceBackend{
										Name: robotVDI.GetRobotVDIServiceTCPMetadata().Name,
										Port: networkingv1.ServiceBackendPort{
											Number: int32(internal.FILE_BROWSER_PORT),
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

func GetRobotVDICustomService(robotVDI *robotv1alpha1.RobotVDI, svcNamespacedName *types.NamespacedName, robot robotv1alpha1.Robot) *corev1.Service {

	var ports []corev1.ServicePort

	if portsStr, ok := robot.Spec.AdditionalConfigs[internal.VDI_CUSTOM_PORT_RANGE_KEY]; ok {
		portsSlice := strings.Split(portsStr.Value, "/")
		for _, p := range portsSlice {
			portInfo := strings.Split(p, "-")
			portName := portInfo[0]
			fwdStr := strings.Split(portInfo[1], ":")
			nodePortVal, _ := strconv.ParseInt(fwdStr[0], 10, 64)
			containerPortVal, _ := strconv.ParseInt(fwdStr[1], 10, 64)
			ports = append(ports, corev1.ServicePort{
				Port: int32(containerPortVal),
				TargetPort: intstr.IntOrString{
					IntVal: int32(containerPortVal),
				},
				NodePort: int32(nodePortVal),
				Protocol: corev1.ProtocolTCP,
				Name:     portName,
			})
		}
	}

	serviceSpec := corev1.ServiceSpec{
		Type:     corev1.ServiceTypeNodePort, // robotVDI.Spec.ServiceType,
		Selector: getRobotVDISelector(*robotVDI),
		Ports:    ports,
	}

	service := corev1.Service{
		ObjectMeta: metav1.ObjectMeta{
			Name:      svcNamespacedName.Name,
			Namespace: svcNamespacedName.Namespace,
		},
		Spec: serviceSpec,
	}

	return &service
}

func GetRobotVDICustomIngress(robotVDI *robotv1alpha1.RobotVDI, ingressNamespacedName *types.NamespacedName, robot robotv1alpha1.Robot) *networkingv1.Ingress {

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

	var ingressPaths []networkingv1.HTTPIngressPath

	if portsStr, ok := robot.Spec.AdditionalConfigs[internal.VDI_CUSTOM_PORT_RANGE_KEY]; ok {
		portsSlice := strings.Split(portsStr.Value, "/")
		for _, p := range portsSlice {
			portInfo := strings.Split(p, "-")
			portName := portInfo[0]
			fwdStr := strings.Split(portInfo[1], ":")
			// nodePortVal, _ := strconv.ParseInt(fwdStr[0], 10, 64)
			containerPortVal, _ := strconv.ParseInt(fwdStr[1], 10, 64)
			ingressPaths = append(ingressPaths, networkingv1.HTTPIngressPath{
				Path:     robotv1alpha1.GetRobotServicePath(robot, "/custom/vdi/"+portName) + "(/|$)(.*)",
				PathType: &pathTypePrefix,
				Backend: networkingv1.IngressBackend{
					Service: &networkingv1.IngressServiceBackend{
						Name: robotVDI.GetRobotVDICustomServiceMetadata().Name,
						Port: networkingv1.ServiceBackendPort{
							Number: int32(containerPortVal),
						},
					},
				},
			})
		}
	}

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
						Paths: ingressPaths,
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

func getResourceLimits(resources robotv1alpha1.Resources) corev1.ResourceList {
	resourceLimits := corev1.ResourceList{}
	if resources.GPUCore != 0 {
		resourceLimits[corev1.ResourceName(resources.GPUInstance)] = resource.MustParse(strconv.Itoa(resources.GPUCore))
	}
	if resources.CPU != "" {
		resourceLimits["cpu"] = resource.MustParse(resources.CPU)
	}
	if resources.Memory != "" {
		resourceLimits["memory"] = resource.MustParse(resources.Memory)
	}

	return resourceLimits
}
