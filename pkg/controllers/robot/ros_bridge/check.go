package ros_bridge

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
)

func (r *ROSBridgeReconciler) reconcileCheckOwnedResources(ctx context.Context, instance *robotv1alpha1.ROSBridge) error {

	bridgeServiceQuery := &corev1.Service{}
	err := r.Get(ctx, *instance.GetBridgeServiceMetadata(), bridgeServiceQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.ServiceStatus = robotv1alpha1.BridgeServiceStatus{}
	} else if err != nil {
		return err
	} else {
		instance.Status.ServiceStatus.Created = true
	}

	bridgePodQuery := &corev1.Pod{}
	err = r.Get(ctx, *instance.GetBridgePodMetadata(), bridgePodQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.PodStatus = robotv1alpha1.BridgePodStatus{}
	} else if err != nil {
		return err
	} else {
		instance.Status.PodStatus.Created = true
		instance.Status.PodStatus.Phase = bridgePodQuery.Status.Phase
	}

	return nil
}
