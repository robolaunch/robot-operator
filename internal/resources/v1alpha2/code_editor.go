package v1alpha2_resources

import (
	appsv1 "k8s.io/api/apps/v1"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"

	"github.com/robolaunch/robot-operator/internal"
	configure "github.com/robolaunch/robot-operator/internal/configure/v1alpha2"
	"github.com/robolaunch/robot-operator/internal/label"
	"github.com/robolaunch/robot-operator/internal/platform"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
)

const (
	CODE_EDITOR_PORT_NAME = "code-server"
	CODE_EDITOR_APP_NAME  = "code-editor"
)

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

	image, err := platform.GetToolsImage(platformMeta.Version, CODE_EDITOR_APP_NAME, codeEditor.Spec.Version)
	if err != nil {
		return nil
	}

	podSpec := corev1.PodSpec{
		Containers: []corev1.Container{
			{
				Name:    CODE_EDITOR_APP_NAME,
				Image:   image,
				Command: []string{"/bin/bash", "-c", "sleep infinity"},
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

	deployment := appsv1.Deployment{
		ObjectMeta: metav1.ObjectMeta{
			Name:      deploymentNamespacedName.Name,
			Namespace: deploymentNamespacedName.Namespace,
			Labels:    codeEditor.Labels,
		},
		Spec: appsv1.DeploymentSpec{
			Selector: &metav1.LabelSelector{
				MatchLabels: map[string]string{
					internal.CODE_EDITOR_CONTAINER_SELECTOR_LABEL_KEY: CODE_EDITOR_APP_NAME,
				},
			},
			Template: corev1.PodTemplateSpec{
				ObjectMeta: metav1.ObjectMeta{
					Labels: map[string]string{
						internal.CODE_EDITOR_CONTAINER_SELECTOR_LABEL_KEY: CODE_EDITOR_APP_NAME,
					},
				},
				Spec: podSpec,
			},
		},
	}

	return &deployment
}
