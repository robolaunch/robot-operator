package relay_server

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	"k8s.io/apimachinery/pkg/types"
)

func (r *RelayServerReconciler) createPod(ctx context.Context, instance *robotv1alpha1.RelayServer, podNamespacedName *types.NamespacedName) error {
	return nil
}

func (r *RelayServerReconciler) createService(ctx context.Context, instance *robotv1alpha1.RelayServer, svcNamespacedName *types.NamespacedName) error {
	return nil
}

func (r *RelayServerReconciler) createIngress(ctx context.Context, instance *robotv1alpha1.RelayServer, ingressNamespacedName *types.NamespacedName) error {
	return nil
}
