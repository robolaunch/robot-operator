package discovery_server

import (
	"context"

	robotErr "github.com/robolaunch/robot-operator/internal/error"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
)

func (r *DiscoveryServerReconciler) reconcileHandleService(ctx context.Context, instance *robotv1alpha1.DiscoveryServer) error {

	if !instance.Status.ServiceStatus.Created {
		instance.Status.Phase = robotv1alpha1.DiscoveryServerPhaseCreatingService
		err := r.createService(ctx, instance, instance.GetDiscoveryServerServiceMetadata())
		if err != nil {
			return err
		}
		instance.Status.ServiceStatus.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "Service",
			ResourceName:      instance.GetDiscoveryServerPodMetadata().Name,
			ResourceNamespace: instance.GetDiscoveryServerPodMetadata().Namespace,
		}
	}

	return nil
}

func (r *DiscoveryServerReconciler) reconcileHandlePod(ctx context.Context, instance *robotv1alpha1.DiscoveryServer) error {

	if !instance.Status.PodStatus.Resource.Created {
		instance.Status.Phase = robotv1alpha1.DiscoveryServerPhaseCreatingPod
		err := r.createPod(ctx, instance, instance.GetDiscoveryServerPodMetadata())
		if err != nil {
			return err
		}
		instance.Status.PodStatus.Resource.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "Pod",
			ResourceName:      instance.GetDiscoveryServerPodMetadata().Name,
			ResourceNamespace: instance.GetDiscoveryServerPodMetadata().Namespace,
		}
	}

	// TODO: Cover other pod phases
	if instance.Status.PodStatus.Resource.Phase != string(corev1.PodRunning) {
		return &robotErr.WaitingForResourceError{
			ResourceKind:      "Pod",
			ResourceName:      instance.GetDiscoveryServerPodMetadata().Name,
			ResourceNamespace: instance.GetDiscoveryServerPodMetadata().Namespace,
		}
	}

	return nil
}

func (r *DiscoveryServerReconciler) reconcileHandleServiceExport(ctx context.Context, instance *robotv1alpha1.DiscoveryServer) error {

	if !instance.Status.ServiceExportStatus.Created {
		instance.Status.Phase = robotv1alpha1.DiscoveryServerPhaseCreatingServiceExport
		err := r.createServiceExport(ctx, instance, instance.GetDiscoveryServerServiceMetadata())
		if err != nil {
			return err
		}
		instance.Status.ServiceExportStatus.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "ServiceExport",
			ResourceName:      instance.GetDiscoveryServerServiceMetadata().Name,
			ResourceNamespace: instance.GetDiscoveryServerServiceMetadata().Namespace,
		}
	}

	return nil
}

func (r *DiscoveryServerReconciler) reconcileHandleConfigMap(ctx context.Context, instance *robotv1alpha1.DiscoveryServer) error {

	if !instance.Status.ConfigMapStatus.Created {

		instance.Status.Phase = robotv1alpha1.DiscoveryServerPhaseCreatingConfigMap

		if instance.Status.ConnectionInfo.IP != "" {

			err := r.createConfigMap(ctx, instance, instance.GetDiscoveryServerConfigMapMetadata())
			if err != nil {
				return err
			}
			instance.Status.ConfigMapStatus.Created = true

		}
	}

	return nil
}
