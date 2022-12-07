package ros_bridge

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
)

func (r *ROSBridgeReconciler) reconcileCheckOwnedResources(ctx context.Context, instance *robotv1alpha1.ROSBridge) error {
	return nil
}

func (r *ROSBridgeReconciler) reconcileUpdateConnectionInfo(ctx context.Context, instance *robotv1alpha1.ROSBridge) error {
	return nil
}
