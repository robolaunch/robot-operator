package v1alpha2_resources

import (
	"fmt"
	"path/filepath"
	"strconv"
	"strings"

	appsv1 "k8s.io/api/apps/v1"
	corev1 "k8s.io/api/core/v1"
	networkingv1 "k8s.io/api/networking/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/apimachinery/pkg/util/intstr"

	"github.com/robolaunch/robot-operator/internal"
	configure "github.com/robolaunch/robot-operator/internal/configure/v1alpha2"
	"github.com/robolaunch/robot-operator/internal/label"
	"github.com/robolaunch/robot-operator/internal/platform"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
)

func getCodeEditorSelector(codeEditor robotv1alpha2.CodeEditor) map[string]string {
	return map[string]string{
		internal.CODE_EDITOR_SELECTOR_LABEL_KEY: codeEditor.Name,
	}
}

func GetCodeEditorPersistentVolumeClaim(codeEditor *robotv1alpha2.CodeEditor, pvcNamespacedName *types.NamespacedName, key int) *corev1.PersistentVolumeClaim {

	pvc := corev1.PersistentVolumeClaim{
		ObjectMeta: metav1.ObjectMeta{
			Name:      pvcNamespacedName.Name,
			Namespace: pvcNamespacedName.Namespace,
			Labels:    codeEditor.Labels,
		},
		Spec: codeEditor.Spec.VolumeClaimTemplates[key].Spec,
	}

	return &pvc

}

func GetCodeEditorDeployment(codeEditor *robotv1alpha2.CodeEditor, deploymentNamespacedName *types.NamespacedName, node corev1.Node) *appsv1.Deployment {

	platformMeta := label.GetPlatformMeta(&node)

	cfg := configure.PodSpecConfigInjector{}

	image, err := platform.GetToolsImage(codeEditor, platformMeta.Version, internal.CODE_EDITOR_APP_NAME, codeEditor.Spec.Version)
	if err != nil {
		return nil
	}

	var cmdBuilder strings.Builder
	cmdBuilder.WriteString("supervisord -c " + filepath.Join("/etc", "robolaunch", "services", "code-server.conf"))

	podSpec := corev1.PodSpec{
		Containers: []corev1.Container{
			{
				Name:    internal.CODE_EDITOR_APP_NAME,
				Image:   image,
				Command: internal.Bash(cmdBuilder.String()),
				Env: []corev1.EnvVar{
					internal.Env("CODE_SERVER_PORT", strconv.FormatInt(int64(codeEditor.Spec.Port), 10)),
					internal.Env("FILE_BROWSER_PORT", strconv.Itoa(internal.FILE_BROWSER_PORT)),
					internal.Env("FILE_BROWSER_SERVICE", "code-server"),
					internal.Env("FILE_BROWSER_BASE_URL", robotv1alpha1.GetServicePath(codeEditor, "/"+internal.FILE_BROWSER_PORT_NAME)),
					internal.Env("TERM", "xterm-256color"),
				},
				Ports: []corev1.ContainerPort{
					{
						Name:          internal.CODE_EDITOR_PORT_NAME,
						ContainerPort: codeEditor.Spec.Port,
						Protocol:      corev1.ProtocolTCP,
					},
					{
						Name:          internal.FILE_BROWSER_PORT_NAME,
						ContainerPort: internal.FILE_BROWSER_PORT,
						Protocol:      corev1.ProtocolTCP,
					},
				},
				SecurityContext: &codeEditor.Spec.Container.SecurityContext,
				VolumeMounts:    codeEditor.Spec.Container.VolumeMounts,
			},
		},
	}

	cfg.InjectImagePullPolicy(&podSpec)
	cfg.SchedulePod(&podSpec, codeEditor)
	cfg.InjectTimezone(&podSpec, node)
	cfg.InjectRuntimeClass(&podSpec, codeEditor, node)
	cfg.InjectVolumeConfiguration(&podSpec, codeEditor.Status.PVCStatuses)
	cfg.InjectExternalVolumeConfiguration(&podSpec, codeEditor.Status.ExternalVolumeStatuses)

	if codeEditor.Spec.Remote {
		cfg.InjectRemoteConfigurations(&podSpec, codeEditor)
	}

	deployment := appsv1.Deployment{
		ObjectMeta: metav1.ObjectMeta{
			Name:      deploymentNamespacedName.Name,
			Namespace: deploymentNamespacedName.Namespace,
			Labels:    codeEditor.Labels,
		},
		Spec: appsv1.DeploymentSpec{
			Selector: &metav1.LabelSelector{
				MatchLabels: getCodeEditorSelector(*codeEditor),
			},
			Template: corev1.PodTemplateSpec{
				ObjectMeta: metav1.ObjectMeta{
					Labels: getCodeEditorSelector(*codeEditor),
				},
				Spec: podSpec,
			},
		},
	}

	return &deployment
}

func GetCodeEditorService(codeEditor *robotv1alpha2.CodeEditor, svcNamespacedName *types.NamespacedName) *corev1.Service {

	cfg := configure.ServiceSpecConfigInjector{}

	serviceSpec := corev1.ServiceSpec{
		Type:     codeEditor.Spec.ServiceType,
		Selector: getCodeEditorSelector(*codeEditor),
		Ports: []corev1.ServicePort{
			{
				Name: internal.CODE_EDITOR_PORT_NAME,
				Port: codeEditor.Spec.Port,
				TargetPort: intstr.IntOrString{
					IntVal: codeEditor.Spec.Port,
				},
				Protocol: corev1.ProtocolTCP,
			},
			{
				Name: internal.FILE_BROWSER_PORT_NAME,
				Port: internal.FILE_BROWSER_PORT,
				TargetPort: intstr.IntOrString{
					IntVal: internal.FILE_BROWSER_PORT,
				},
				Protocol: corev1.ProtocolTCP,
			},
		},
	}

	if codeEditor.Spec.Remote {
		cfg.InjectRemoteConfigurations(&serviceSpec)
	}

	service := corev1.Service{
		ObjectMeta: metav1.ObjectMeta{
			Name:      svcNamespacedName.Name,
			Namespace: svcNamespacedName.Namespace,
			Labels:    codeEditor.Labels,
		},
		Spec: serviceSpec,
	}

	return &service
}

func GetCodeEditorIngress(codeEditor *robotv1alpha2.CodeEditor, ingressNamespacedName *types.NamespacedName) *networkingv1.Ingress {

	tenancy := label.GetTenancy(codeEditor)
	platformMeta := label.GetPlatformMeta(codeEditor)

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
				SecretName: codeEditor.Spec.TLSSecretName,
			},
		},
		Rules: []networkingv1.IngressRule{
			{
				Host: tenancy.Team + "." + platformMeta.Domain,
				IngressRuleValue: networkingv1.IngressRuleValue{
					HTTP: &networkingv1.HTTPIngressRuleValue{
						Paths: []networkingv1.HTTPIngressPath{
							{
								Path:     robotv1alpha1.GetServicePath(codeEditor, "/"+internal.CODE_EDITOR_APP_NAME) + "(/|$)(.*)",
								PathType: &pathTypePrefix,
								Backend: networkingv1.IngressBackend{
									Service: &networkingv1.IngressServiceBackend{
										Name: codeEditor.GetServiceMetadata().Name,
										Port: networkingv1.ServiceBackendPort{
											Number: codeEditor.Spec.Port,
										},
									},
								},
							},
							{
								Path:     robotv1alpha1.GetServicePath(codeEditor, "/"+internal.FILE_BROWSER_PORT_NAME) + "(/|$)(.*)",
								PathType: &pathTypePrefix,
								Backend: networkingv1.IngressBackend{
									Service: &networkingv1.IngressServiceBackend{
										Name: codeEditor.GetServiceMetadata().Name,
										Port: networkingv1.ServiceBackendPort{
											Number: internal.FILE_BROWSER_PORT,
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
			Labels:      codeEditor.Labels,
			Annotations: annotations,
		},
		Spec: ingressSpec,
	}

	return ingress
}
