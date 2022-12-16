package ros_bridge

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/roboscale.io/v1alpha1"
	"github.com/robolaunch/robot-operator/internal/resources"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
)

func (r *ROSBridgeReconciler) reconcileCheckService(ctx context.Context, instance *robotv1alpha1.ROSBridge) error {

	bridgeServiceQuery := &corev1.Service{}
	err := r.Get(ctx, *instance.GetBridgeServiceMetadata(), bridgeServiceQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.ServiceStatus = robotv1alpha1.BridgeServiceStatus{}
	} else if err != nil {
		return err
	} else {
		instance.Status.ServiceStatus.Created = true
	}

	return nil
}

func (r *ROSBridgeReconciler) reconcileCheckPod(ctx context.Context, instance *robotv1alpha1.ROSBridge) error {

	bridgePodQuery := &corev1.Pod{}
	err := r.Get(ctx, *instance.GetBridgePodMetadata(), bridgePodQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.PodStatus = robotv1alpha1.BridgePodStatus{}
	} else if err != nil {
		return err
	} else {

		rosBridgeFound := false
		ros2BridgeFound := false
		rightImage := true
		for _, container := range bridgePodQuery.Spec.Containers {
			if container.Name == resources.ROS_BRIDGE_PORT_NAME {
				rosBridgeFound = true
			}
			if container.Name == resources.ROS2_BRIDGE_PORT_NAME {
				ros2BridgeFound = true
			}
			if instance.Spec.Image != container.Image {
				rightImage = false
			}
		}

		if (instance.Spec.ROS.Enabled && !rosBridgeFound) ||
			(instance.Spec.ROS2.Enabled && !ros2BridgeFound) ||
			!rightImage {
			err = r.Delete(ctx, bridgePodQuery)
			if err != nil {
				return err
			}

			instance.Status.PodStatus.Phase = ""
			return nil
		}

		instance.Status.PodStatus.Created = true
		instance.Status.PodStatus.Phase = bridgePodQuery.Status.Phase
	}

	return nil
}
