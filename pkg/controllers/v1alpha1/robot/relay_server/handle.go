package relay_server

import (
	"context"

	robotErr "github.com/robolaunch/robot-operator/internal/error"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
)

func (r *RelayServerReconciler) reconcileHandlePod(ctx context.Context, instance *robotv1alpha1.RelayServer) error {

	if !instance.Status.PodStatus.Created {
		instance.Status.Phase = robotv1alpha1.RelayServerPhaseCreatingPod
		err := r.createPod(ctx, instance, instance.GetRelayServerPodMetadata())
		if err != nil {
			return err
		}
		instance.Status.PodStatus.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "Pod",
			ResourceName:      instance.GetRelayServerPodMetadata().Name,
			ResourceNamespace: instance.GetRelayServerPodMetadata().Namespace,
		}
	}

	if instance.Status.PodStatus.Phase != string(corev1.PodRunning) {
		return &robotErr.WaitingForResourceError{
			ResourceKind:      "Pod",
			ResourceName:      instance.GetRelayServerPodMetadata().Name,
			ResourceNamespace: instance.GetRelayServerPodMetadata().Namespace,
		}
	}

	return nil
}

func (r *RelayServerReconciler) reconcileHandleService(ctx context.Context, instance *robotv1alpha1.RelayServer) error {

	if !instance.Status.ServiceStatus.Resource.Created {
		instance.Status.Phase = robotv1alpha1.RelayServerPhaseCreatingService
		err := r.createService(ctx, instance, instance.GetRelayServerServiceMetadata())
		if err != nil {
			return err
		}
		instance.Status.ServiceStatus.Resource.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "Service",
			ResourceName:      instance.GetRelayServerServiceMetadata().Name,
			ResourceNamespace: instance.GetRelayServerServiceMetadata().Namespace,
		}
	}

	return nil
}

func (r *RelayServerReconciler) reconcileHandleIngress(ctx context.Context, instance *robotv1alpha1.RelayServer) error {

	if !instance.Status.IngressStatus.Created {
		instance.Status.Phase = robotv1alpha1.RelayServerPhaseCreatingIngress
		err := r.createIngress(ctx, instance, instance.GetRelayServerIngressMetadata())
		if err != nil {
			return err
		}
		instance.Status.IngressStatus.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "Ingress",
			ResourceName:      instance.GetRelayServerIngressMetadata().Name,
			ResourceNamespace: instance.GetRelayServerIngressMetadata().Namespace,
		}
	}

	return nil
}
