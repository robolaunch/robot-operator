package ros_bridge

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	"k8s.io/apimachinery/pkg/types"
)

func (r *ROSBridgeReconciler) createService(ctx context.Context, instance *robotv1alpha1.DiscoveryServer, svcNamespacedName *types.NamespacedName) error {

	return nil
}

func (r *ROSBridgeReconciler) createPod(ctx context.Context, instance *robotv1alpha1.DiscoveryServer, podNamespacedName *types.NamespacedName) error {

	return nil
}
