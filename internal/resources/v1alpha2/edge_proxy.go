package v1alpha2_resources

import (
	"strconv"
	"strings"

	"github.com/robolaunch/robot-operator/internal"
	configure "github.com/robolaunch/robot-operator/internal/configure/v1alpha2"
	"github.com/robolaunch/robot-operator/internal/label"
	"github.com/robolaunch/robot-operator/internal/platform"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	appsv1 "k8s.io/api/apps/v1"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
)

func getEdgeProxySelector(edgeProxy robotv1alpha2.EdgeProxy) map[string]string {
	return map[string]string{
		internal.EDGE_PROXY_SELECTOR_LABEL_KEY: edgeProxy.Name,
	}
}

func GetEdgeProxyDeployment(edgeProxy *robotv1alpha2.EdgeProxy, deploymentNamespacedName *types.NamespacedName, node corev1.Node) *appsv1.Deployment {

	platformMeta := label.GetPlatformMeta(&node)

	cfg := configure.PodSpecConfigInjector{}

	image, err := platform.GetToolsImage(edgeProxy, platformMeta.Version, internal.EDGE_PROXY_APP_NAME, edgeProxy.Spec.Version)
	if err != nil {
		return nil
	}

	var cmdBuilder strings.Builder
	cmdBuilder.WriteString(
		"socat TCP4-LISTEN:" + strconv.Itoa(int(edgeProxy.Spec.RemotePort)) + ",fork,reuseaddr TCP4:" +
			edgeProxy.Spec.Hostname + "." +
			edgeProxy.Spec.Instance + "." +
			edgeProxy.Spec.Subdomain + "." +
			edgeProxy.Namespace +
			".svc.clusterset.local:" +
			strconv.Itoa(int(edgeProxy.Spec.RemotePort)))

	podSpec := corev1.PodSpec{
		Containers: []corev1.Container{
			{
				Name:    internal.EDGE_PROXY_APP_NAME,
				Image:   image,
				Command: internal.Bash(cmdBuilder.String()),
				Ports: []corev1.ContainerPort{
					{
						Name:          "tcp",
						Protocol:      corev1.ProtocolTCP,
						ContainerPort: edgeProxy.Spec.RemotePort,
					},
				},
			},
		},
	}

	cfg.InjectImagePullPolicy(&podSpec)
	cfg.SchedulePod(&podSpec, edgeProxy)

	deployment := appsv1.Deployment{
		ObjectMeta: metav1.ObjectMeta{
			Name:      deploymentNamespacedName.Name,
			Namespace: deploymentNamespacedName.Namespace,
			Labels:    getEdgeProxySelector(*edgeProxy),
		},
		Spec: appsv1.DeploymentSpec{
			Selector: &metav1.LabelSelector{
				MatchLabels: getEdgeProxySelector(*edgeProxy),
			},
			Template: corev1.PodTemplateSpec{
				ObjectMeta: metav1.ObjectMeta{
					Labels: getEdgeProxySelector(*edgeProxy),
				},
				Spec: podSpec,
			},
		},
	}

	return &deployment
}
