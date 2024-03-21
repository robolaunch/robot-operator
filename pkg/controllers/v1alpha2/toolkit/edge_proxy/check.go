package edge_proxy

import (
	"context"
	"reflect"

	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/label"
	"github.com/robolaunch/robot-operator/internal/platform"
	"github.com/robolaunch/robot-operator/internal/reference"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	appsv1 "k8s.io/api/apps/v1"
	corev1 "k8s.io/api/core/v1"
	k8sErr "k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/labels"
	"k8s.io/apimachinery/pkg/selection"
	"sigs.k8s.io/controller-runtime/pkg/client"
)

func (r *EdgeProxyReconciler) reconcileCheckDeployment(ctx context.Context, instance *robotv1alpha2.EdgeProxy) error {

	deploymentQuery := &appsv1.Deployment{}
	err := r.Get(ctx, *instance.GetDeploymentMetadata(), deploymentQuery)
	if err != nil && k8sErr.IsNotFound(err) {
		instance.Status.DeploymentStatus = robotv1alpha2.OwnedDeploymentStatus{}
	} else if err != nil {
		return err
	} else {

		platformMeta := label.GetPlatformMeta(instance)

		desiredImage, err := platform.GetToolsImage(instance, platformMeta.Version, internal.EDGE_PROXY_APP_NAME, instance.Spec.Version)
		if err != nil {
			return err
		}

		actualImage := deploymentQuery.Spec.Template.Spec.Containers[0].Image

		imageSynced := reflect.DeepEqual(desiredImage, actualImage)

		desiredPort := instance.Spec.RemotePort
		var actualPort int32
		if len(deploymentQuery.Spec.Template.Spec.Containers) > 0 {
			cont := deploymentQuery.Spec.Template.Spec.Containers[0]
			for _, cPort := range cont.Ports {
				if cPort.Name == internal.EDGE_PROXY_APP_NAME {
					actualPort = cPort.ContainerPort
				}
			}
		}

		portSynced := reflect.DeepEqual(desiredPort, actualPort)

		if !imageSynced ||
			!portSynced {
			err := r.updateDeployment(ctx, instance)
			if err != nil {
				return err
			}
		}

		// update container statuses
		newReq, err := labels.NewRequirement(internal.EDGE_PROXY_SELECTOR_LABEL_KEY, selection.In, []string{instance.Name})
		if err != nil {
			return err
		}
		podSelector := labels.NewSelector().Add([]labels.Requirement{*newReq}...)

		podList := corev1.PodList{}
		err = r.List(ctx, &podList, &client.ListOptions{
			LabelSelector: podSelector,
		})
		if err != nil && k8sErr.IsNotFound(err) {
			instance.Status.DeploymentStatus.ContainerStatuses = []corev1.ContainerStatus{}
		} else if err != nil {
			return err
		} else {
			containerStatuses := []corev1.ContainerStatus{}
			for _, pod := range podList.Items {
				containerStatuses = append(containerStatuses, pod.Status.ContainerStatuses...)
			}
			instance.Status.DeploymentStatus.ContainerStatuses = containerStatuses
		}

		instance.Status.DeploymentStatus.Resource.Created = true
		reference.SetReference(&instance.Status.DeploymentStatus.Resource.Reference, deploymentQuery.TypeMeta, deploymentQuery.ObjectMeta)
	}

	return nil
}
