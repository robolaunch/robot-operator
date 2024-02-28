package ros2_bridge

import (
	"context"
	"reflect"
	"strconv"

	"github.com/robolaunch/robot-operator/internal/handle"
	"github.com/robolaunch/robot-operator/internal/reference"
	"github.com/robolaunch/robot-operator/internal/resources"
	corev1 "k8s.io/api/core/v1"
	networkingv1 "k8s.io/api/networking/v1"
	"k8s.io/apimachinery/pkg/api/errors"

	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
)

func (r *ROS2BridgeReconciler) reconcileCheckService(ctx context.Context, instance *robotv1alpha2.ROS2Bridge) error {

	bridgeServiceQuery := &corev1.Service{}
	err := r.Get(ctx, *instance.GetROS2BridgeServiceMetadata(), bridgeServiceQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.ServiceStatus = robotv1alpha1.OwnedServiceStatus{}
	} else if err != nil {
		return err
	} else {

		isServiceOk := reflect.DeepEqual(instance.Spec.ServiceType, bridgeServiceQuery.Spec.Type)

		if !isServiceOk {
			err := r.Delete(ctx, bridgeServiceQuery)
			if err != nil {
				return err
			}
			instance.Status.ServiceStatus = robotv1alpha1.OwnedServiceStatus{}
			return nil
		}

		instance.Status.ServiceStatus.Resource.Created = true
		reference.SetReference(&instance.Status.ServiceStatus.Resource.Reference, bridgeServiceQuery.TypeMeta, bridgeServiceQuery.ObjectMeta)
		if instance.Spec.Ingress {
			instance.Status.ServiceStatus.URLs = map[string]string{}
			instance.Status.ServiceStatus.URLs["bridge"] = robotv1alpha1.GetServiceDNS(instance, "wss://", "/bridge")
		} else if instance.Spec.ServiceType == corev1.ServiceTypeNodePort {
			var port int32 = 0
			for _, v := range bridgeServiceQuery.Spec.Ports {
				if v.Name == resources.ROS2_BRIDGE_PORT_NAME {
					port = v.NodePort
				}
			}
			instance.Status.ServiceStatus.URLs = map[string]string{}
			instance.Status.ServiceStatus.URLs["bridge"] = robotv1alpha1.GetServiceDNSWithNodePort(instance, "ws://", strconv.Itoa(int(port)))
		}
	}

	return nil
}

func (r *ROS2BridgeReconciler) reconcileCheckPod(ctx context.Context, instance *robotv1alpha2.ROS2Bridge) error {

	bridgePodQuery := &corev1.Pod{}
	err := r.Get(ctx, *instance.GetROS2BridgePodMetadata(), bridgePodQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.PodStatus = robotv1alpha1.OwnedResourceStatus{}
	} else if err != nil {
		return err
	} else {

		err := handle.HandlePod(ctx, r, *bridgePodQuery)
		if err != nil {
			return err
		}

		instance.Status.PodStatus.Created = true
		reference.SetReference(&instance.Status.PodStatus.Reference, bridgePodQuery.TypeMeta, bridgePodQuery.ObjectMeta)
		instance.Status.PodStatus.Phase = string(bridgePodQuery.Status.Phase)
	}

	return nil
}

func (r *ROS2BridgeReconciler) reconcileCheckIngress(ctx context.Context, instance *robotv1alpha2.ROS2Bridge) error {

	if instance.Spec.Ingress {
		ingressQuery := &networkingv1.Ingress{}
		err := r.Get(ctx, *instance.GetROS2BridgeIngressMetadata(), ingressQuery)
		if err != nil {
			if errors.IsNotFound(err) {
				instance.Status.IngressStatus = robotv1alpha1.OwnedResourceStatus{}
			} else {
				return err
			}
		} else {

			if len(ingressQuery.Spec.TLS) > 0 {
				ingressTLS := ingressQuery.Spec.TLS[0]
				ingressTLS.SecretName = instance.Spec.TLSSecretName
				ingressQuery.Spec.TLS[0] = ingressTLS

				err := r.Update(ctx, ingressQuery)
				if err != nil {
					return err
				}
			}

			instance.Status.IngressStatus.Created = true
			reference.SetReference(&instance.Status.IngressStatus.Reference, ingressQuery.TypeMeta, ingressQuery.ObjectMeta)
		}
	} else {
		ingressQuery := &networkingv1.Ingress{}
		err := r.Get(ctx, *instance.GetROS2BridgeIngressMetadata(), ingressQuery)
		if err != nil {
			if errors.IsNotFound(err) {
				instance.Status.IngressStatus = robotv1alpha1.OwnedResourceStatus{}
			} else {
				return err
			}
		} else {

			err := r.Delete(ctx, ingressQuery)
			if err != nil {
				return err
			}

			instance.Status.IngressStatus = robotv1alpha1.OwnedResourceStatus{}
		}
	}

	return nil
}
