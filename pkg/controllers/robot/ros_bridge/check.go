package ros_bridge

import (
	"context"
	"strconv"

	"github.com/robolaunch/robot-operator/internal/handle"
	"github.com/robolaunch/robot-operator/internal/reference"
	"github.com/robolaunch/robot-operator/internal/resources"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	networkingv1 "k8s.io/api/networking/v1"
	"k8s.io/apimachinery/pkg/api/errors"
)

func (r *ROSBridgeReconciler) reconcileCheckService(ctx context.Context, instance *robotv1alpha1.ROSBridge) error {

	bridgeServiceQuery := &corev1.Service{}
	err := r.Get(ctx, *instance.GetBridgeServiceMetadata(), bridgeServiceQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.ServiceStatus = robotv1alpha1.OwnedServiceStatus{}
	} else if err != nil {
		return err
	} else {
		robot, err := r.reconcileGetOwner(ctx, instance)
		if err != nil {
			return err
		}

		instance.Status.ServiceStatus.Resource.Created = true
		reference.SetReference(&instance.Status.ServiceStatus.Resource.Reference, bridgeServiceQuery.TypeMeta, bridgeServiceQuery.ObjectMeta)
		if instance.Spec.Ingress {
			instance.Status.ServiceStatus.URL = robotv1alpha1.GetRobotServiceDNS(*robot, "wss://", "/bridge")
		} else if instance.Spec.ServiceType == corev1.ServiceTypeNodePort {
			instance.Status.ServiceStatus.URL = robotv1alpha1.GetRobotServiceDNSWithNodePort(*robot, strconv.Itoa(resources.ROS2_BRIDGE_PORT))
		}
	}

	return nil
}

func (r *ROSBridgeReconciler) reconcileCheckPod(ctx context.Context, instance *robotv1alpha1.ROSBridge) error {

	bridgePodQuery := &corev1.Pod{}
	err := r.Get(ctx, *instance.GetBridgePodMetadata(), bridgePodQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.PodStatus = robotv1alpha1.OwnedResourceStatus{}
	} else if err != nil {
		return err
	} else {

		err := handle.HandlePod(ctx, r, *bridgePodQuery)
		if err != nil {
			return err
		}

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
		reference.SetReference(&instance.Status.PodStatus.Reference, bridgePodQuery.TypeMeta, bridgePodQuery.ObjectMeta)
		instance.Status.PodStatus.Phase = string(bridgePodQuery.Status.Phase)
	}

	return nil
}

func (r *ROSBridgeReconciler) reconcileCheckIngress(ctx context.Context, instance *robotv1alpha1.ROSBridge) error {

	if instance.Spec.Ingress {
		ingressQuery := &networkingv1.Ingress{}
		err := r.Get(ctx, *instance.GetBridgeIngressMetadata(), ingressQuery)
		if err != nil {
			if errors.IsNotFound(err) {
				instance.Status.IngressStatus = robotv1alpha1.OwnedResourceStatus{}
			} else {
				return err
			}
		} else {
			instance.Status.IngressStatus.Created = true
			reference.SetReference(&instance.Status.IngressStatus.Reference, ingressQuery.TypeMeta, ingressQuery.ObjectMeta)
		}
	}

	return nil
}
