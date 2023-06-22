package relay_server

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
)

func (r *RelayServerReconciler) reconcileCheckPod(ctx context.Context, instance *robotv1alpha1.RelayServer) error {
	return nil
}

func (r *RelayServerReconciler) reconcileCheckService(ctx context.Context, instance *robotv1alpha1.RelayServer) error {
	return nil
}

func (r *RelayServerReconciler) reconcileCheckIngress(ctx context.Context, instance *robotv1alpha1.RelayServer) error {
	return nil
}
