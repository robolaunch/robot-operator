package ros2_bridge

import (
	"context"

	robotErr "github.com/robolaunch/robot-operator/internal/error"
	"github.com/robolaunch/robot-operator/internal/label"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/labels"
	"k8s.io/apimachinery/pkg/selection"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/client-go/util/retry"
	"sigs.k8s.io/controller-runtime/pkg/client"
)

func (r *ROS2BridgeReconciler) reconcileGetInstance(ctx context.Context, meta types.NamespacedName) (*robotv1alpha2.ROS2Bridge, error) {
	instance := &robotv1alpha2.ROS2Bridge{}
	err := r.Get(ctx, meta, instance)
	if err != nil {
		return &robotv1alpha2.ROS2Bridge{}, err
	}

	return instance, nil
}

func (r *ROS2BridgeReconciler) reconcileUpdateInstanceStatus(ctx context.Context, instance *robotv1alpha2.ROS2Bridge) error {
	return retry.RetryOnConflict(retry.DefaultRetry, func() error {
		instanceLV := &robotv1alpha2.ROS2Bridge{}
		err := r.Get(ctx, types.NamespacedName{
			Name:      instance.Name,
			Namespace: instance.Namespace,
		}, instanceLV)

		if err == nil {
			instance.ResourceVersion = instanceLV.ResourceVersion
		}

		err1 := r.Status().Update(ctx, instance)
		return err1
	})
}

func (r *ROS2BridgeReconciler) reconcileGetNode(ctx context.Context, instance *robotv1alpha2.ROS2Bridge) (*corev1.Node, error) {

	tenancyMap := label.GetTenancyMap(instance)

	requirements := []labels.Requirement{}
	for k, v := range tenancyMap {
		newReq, err := labels.NewRequirement(k, selection.In, []string{v})
		if err != nil {
			return nil, err
		}
		requirements = append(requirements, *newReq)
	}

	nodeSelector := labels.NewSelector().Add(requirements...)

	nodes := &corev1.NodeList{}
	err := r.List(ctx, nodes, &client.ListOptions{
		LabelSelector: nodeSelector,
	})
	if err != nil {
		return nil, err
	}

	if len(nodes.Items) == 0 {
		return nil, &robotErr.NodeNotFoundError{
			ResourceKind:      instance.Kind,
			ResourceName:      instance.Name,
			ResourceNamespace: instance.Namespace,
		}
	} else if len(nodes.Items) > 1 {
		return nil, &robotErr.MultipleNodeFoundError{
			ResourceKind:      instance.Kind,
			ResourceName:      instance.Name,
			ResourceNamespace: instance.Namespace,
		}
	}

	return &nodes.Items[0], nil
}

func (r *ROS2BridgeReconciler) reconcileGetDiscoveryServer(ctx context.Context, instance *robotv1alpha2.ROS2Bridge) (*robotv1alpha1.DiscoveryServer, error) {

	ds := &robotv1alpha1.DiscoveryServer{}

	err := r.Get(ctx, types.NamespacedName{
		Namespace: instance.Spec.DiscoveryServerReference.Namespace,
		Name:      instance.Spec.DiscoveryServerReference.Name,
	}, ds)

	if err != nil {
		return nil, err
	}

	return ds, nil
}
