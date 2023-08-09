package ros_bridge

import (
	"context"

	robotErr "github.com/robolaunch/robot-operator/internal/error"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
)

func (r *ROSBridgeReconciler) reconcileHandleService(ctx context.Context, instance *robotv1alpha1.ROSBridge) error {

	if !instance.Status.ServiceStatus.Resource.Created {
		instance.Status.Phase = robotv1alpha1.BridgePhaseCreatingService
		err := r.createService(ctx, instance, instance.GetBridgeServiceMetadata())
		if err != nil {
			return err
		}
		instance.Status.ServiceStatus.Resource.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "Service",
			ResourceName:      instance.GetBridgeServiceMetadata().Name,
			ResourceNamespace: instance.GetBridgeServiceMetadata().Namespace,
		}
	}

	return nil
}

func (r *ROSBridgeReconciler) reconcileHandlePod(ctx context.Context, instance *robotv1alpha1.ROSBridge) error {

	if !instance.Status.PodStatus.Created {
		instance.Status.Phase = robotv1alpha1.BridgePhaseCreatingPod
		err := r.createPod(ctx, instance, instance.GetBridgePodMetadata())
		if err != nil {
			return err
		}
		instance.Status.PodStatus.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "Pod",
			ResourceName:      instance.GetBridgePodMetadata().Name,
			ResourceNamespace: instance.GetBridgePodMetadata().Namespace,
		}
	}

	if instance.Status.PodStatus.Phase != string(corev1.PodRunning) {
		return &robotErr.WaitingForResourceError{
			ResourceKind:      "Pod",
			ResourceName:      instance.GetBridgePodMetadata().Name,
			ResourceNamespace: instance.GetBridgePodMetadata().Namespace,
		}
	}

	return nil
}

func (r *ROSBridgeReconciler) reconcileHandleIngress(ctx context.Context, instance *robotv1alpha1.ROSBridge) error {

	if instance.Spec.Ingress {
		if !instance.Status.IngressStatus.Created {
			instance.Status.Phase = robotv1alpha1.BridgePhaseCreatingIngress
			err := r.createIngress(ctx, instance, instance.GetBridgeIngressMetadata())
			if err != nil {
				return err
			}
			instance.Status.IngressStatus.Created = true

			return &robotErr.CreatingResourceError{
				ResourceKind:      "Ingress",
				ResourceName:      instance.GetBridgeIngressMetadata().Name,
				ResourceNamespace: instance.GetBridgeIngressMetadata().Namespace,
			}
		}
	}

	return nil
}
