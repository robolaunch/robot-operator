package ros2_bridge

import (
	"context"

	"github.com/robolaunch/robot-operator/internal/node"
	v1alpha2_resources "github.com/robolaunch/robot-operator/internal/resources/v1alpha2"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	"k8s.io/apimachinery/pkg/api/errors"
	ctrl "sigs.k8s.io/controller-runtime"
)

func (r *ROS2BridgeReconciler) createService(ctx context.Context, instance *robotv1alpha2.ROS2Bridge) error {

	svc := v1alpha2_resources.GetROS2BridgeService(instance, instance.GetROS2BridgeServiceMetadata())

	err := ctrl.SetControllerReference(instance, svc, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, svc)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Service " + svc.Name + " is created.")
	return nil
}

func (r *ROS2BridgeReconciler) createPod(ctx context.Context, instance *robotv1alpha2.ROS2Bridge) error {

	activeNode, err := r.reconcileGetNode(ctx, instance)
	if err != nil {
		return err
	}

	image, err := node.GetBridgeImage(ctx, r.Client, *activeNode, *instance)
	if err != nil {
		return err
	}

	discoveryServer, err := r.reconcileGetDiscoveryServer(ctx, instance)
	if err != nil {
		return err
	}

	pod := v1alpha2_resources.GetROS2BridgePod(instance, instance.GetROS2BridgePodMetadata(), image, *discoveryServer)

	err = ctrl.SetControllerReference(instance, pod, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, pod)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Pod " + pod.Name + " is created.")
	return nil
}

func (r *ROS2BridgeReconciler) createIngress(ctx context.Context, instance *robotv1alpha2.ROS2Bridge) error {

	ingress := v1alpha2_resources.GetROS2BridgeIngress(instance, instance.GetROS2BridgeIngressMetadata())

	err := ctrl.SetControllerReference(instance, ingress, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, ingress)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Ingress " + ingress.Name + " is created.")
	return nil
}
