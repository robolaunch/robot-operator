package ros2_bridge

import (
	"context"
	"reflect"

	robotErr "github.com/robolaunch/robot-operator/internal/error"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
)

func (r *ROS2BridgeReconciler) reconcileHandleConnectionInfo(ctx context.Context, instance *robotv1alpha2.ROS2Bridge) error {

	if !reflect.DeepEqual(instance.Spec.DiscoveryServerReference, corev1.ObjectReference{}) {
		ds, err := r.reconcileGetDiscoveryServer(ctx, instance)
		if err != nil && errors.IsNotFound(err) {
			instance.Status.ConnectionInfo = robotv1alpha1.ConnectionInfo{}
		} else if err != nil {
			return err
		} else {

			isPodOk := reflect.DeepEqual(instance.Status.ConnectionInfo, ds.Status.ConnectionInfo)

			if !isPodOk && instance.Status.PodStatus.Created {
				bridgePod := &corev1.Pod{}
				err := r.Get(ctx, *instance.GetROS2BridgePodMetadata(), bridgePod)
				if err != nil {
					return err
				}

				err = r.Delete(ctx, bridgePod)
				if err != nil {
					return err
				}
			}

			instance.Status.ConnectionInfo = ds.Status.ConnectionInfo
		}
	}

	return nil
}

func (r *ROS2BridgeReconciler) reconcileHandleService(ctx context.Context, instance *robotv1alpha2.ROS2Bridge) error {

	if !instance.Status.ServiceStatus.Resource.Created {
		instance.Status.Phase = robotv1alpha2.ROS2BridgePhaseCreatingService
		err := r.createService(ctx, instance)
		if err != nil {
			return err
		}
		instance.Status.ServiceStatus.Resource.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "Service",
			ResourceName:      instance.GetROS2BridgeServiceMetadata().Name,
			ResourceNamespace: instance.GetROS2BridgeServiceMetadata().Namespace,
		}
	}

	return nil
}

func (r *ROS2BridgeReconciler) reconcileHandlePod(ctx context.Context, instance *robotv1alpha2.ROS2Bridge) error {

	if !instance.Status.PodStatus.Created {
		instance.Status.Phase = robotv1alpha2.ROS2BridgePhaseCreatingPod
		err := r.createPod(ctx, instance)
		if err != nil {
			return err
		}
		instance.Status.PodStatus.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "Pod",
			ResourceName:      instance.GetROS2BridgePodMetadata().Name,
			ResourceNamespace: instance.GetROS2BridgePodMetadata().Namespace,
		}
	}

	if instance.Status.PodStatus.Phase != string(corev1.PodRunning) {
		return &robotErr.WaitingForResourceError{
			ResourceKind:      "Pod",
			ResourceName:      instance.GetROS2BridgePodMetadata().Name,
			ResourceNamespace: instance.GetROS2BridgePodMetadata().Namespace,
		}
	}

	return nil
}

func (r *ROS2BridgeReconciler) reconcileHandleIngress(ctx context.Context, instance *robotv1alpha2.ROS2Bridge) error {

	if instance.Spec.Ingress {
		if !instance.Status.IngressStatus.Created {
			instance.Status.Phase = robotv1alpha2.ROS2BridgePhaseCreatingIngress
			err := r.createIngress(ctx, instance)
			if err != nil {
				return err
			}
			instance.Status.IngressStatus.Created = true

			return &robotErr.CreatingResourceError{
				ResourceKind:      "Ingress",
				ResourceName:      instance.GetROS2BridgeIngressMetadata().Name,
				ResourceNamespace: instance.GetROS2BridgeIngressMetadata().Namespace,
			}
		}
	}

	return nil
}
