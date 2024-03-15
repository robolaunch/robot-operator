package v1alpha2_resources

import (
	"path/filepath"
	"strconv"
	"strings"

	appsv1 "k8s.io/api/apps/v1"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/apimachinery/pkg/util/intstr"

	"github.com/robolaunch/robot-operator/internal"
	configure "github.com/robolaunch/robot-operator/internal/configure/v1alpha2"
	"github.com/robolaunch/robot-operator/internal/label"
	"github.com/robolaunch/robot-operator/internal/platform"
	"github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
)

const (
	CODE_EDITOR_APP_NAME  = "code-editor"
	CODE_EDITOR_PORT_NAME = "code-server"
)

func getCodeEditorSelector(codeEditor robotv1alpha2.CodeEditor) map[string]string {
	return map[string]string{
		internal.CODE_EDITOR_CONTAINER_SELECTOR_LABEL_KEY: codeEditor.Name,
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

	image, err := platform.GetToolsImage(codeEditor, platformMeta.Version, CODE_EDITOR_APP_NAME, codeEditor.Spec.Version)
	if err != nil {
		return nil
	}

	var cmdBuilder strings.Builder
	cmdBuilder.WriteString("supervisord -c " + filepath.Join("/etc", "robolaunch", "services", "code-server.conf"))

	podSpec := corev1.PodSpec{
		Containers: []corev1.Container{
			{
				Name:    CODE_EDITOR_APP_NAME,
				Image:   image,
				Command: internal.Bash(cmdBuilder.String()),
				Env: []corev1.EnvVar{
					internal.Env("CODE_SERVER_PORT", strconv.FormatInt(int64(codeEditor.Spec.Port), 10)),
					internal.Env("FILE_BROWSER_PORT", strconv.Itoa(internal.FILE_BROWSER_PORT)),
					internal.Env("FILE_BROWSER_SERVICE", CODE_EDITOR_PORT_NAME),
					internal.Env("FILE_BROWSER_BASE_URL", v1alpha1.GetServicePath(codeEditor, "/file-browser/ide")),
					internal.Env("TERM", "xterm-256color"),
				},
				Ports: []corev1.ContainerPort{
					{
						Name:          CODE_EDITOR_PORT_NAME,
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

	if !codeEditor.Spec.Root {
		cfg.InjectLinuxUserAndGroup(&podSpec)
	}

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
				Name: CODE_EDITOR_PORT_NAME,
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

	service := corev1.Service{
		ObjectMeta: metav1.ObjectMeta{
			Name:      svcNamespacedName.Name,
			Namespace: svcNamespacedName.Namespace,
		},
		Spec: serviceSpec,
	}

	if codeEditor.Spec.Remote {
		cfg.InjectRemoteConfigurations(&serviceSpec)
	}

	return &service
}
