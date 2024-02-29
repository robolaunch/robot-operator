package relay_server

import (
	"context"

	resources "github.com/robolaunch/robot-operator/internal/resources/v1alpha1"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/types"
	ctrl "sigs.k8s.io/controller-runtime"
)

func (r *RelayServerReconciler) createPod(ctx context.Context, instance *robotv1alpha1.RelayServer, podNamespacedName *types.NamespacedName) error {

	rsPod := resources.GetRelayServerPod(instance, instance.GetRelayServerPodMetadata())

	err := ctrl.SetControllerReference(instance, rsPod, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, rsPod)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Relay server pod is created.")

	return nil
}

func (r *RelayServerReconciler) createService(ctx context.Context, instance *robotv1alpha1.RelayServer, svcNamespacedName *types.NamespacedName) error {

	rsSvc := resources.GetRelayServerService(instance, instance.GetRelayServerServiceMetadata())

	err := ctrl.SetControllerReference(instance, rsSvc, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, rsSvc)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Relay server service is created.")

	return nil
}

func (r *RelayServerReconciler) createIngress(ctx context.Context, instance *robotv1alpha1.RelayServer, ingressNamespacedName *types.NamespacedName) error {

	rsIngress := resources.GetRelayServerIngress(instance, instance.GetRelayServerIngressMetadata())

	err := ctrl.SetControllerReference(instance, rsIngress, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, rsIngress)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Relay server ingress is created.")

	return nil
}
