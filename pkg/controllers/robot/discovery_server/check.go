package discovery_server

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
)

func (r *DiscoveryServerReconciler) reconcileCheckOwnedResources(ctx context.Context, instance *robotv1alpha1.DiscoveryServer) error {

	discoveryServerServiceQuery := &corev1.Service{}
	err := r.Get(ctx, *instance.GetDiscoveryServerServiceMetadata(), discoveryServerServiceQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.ServiceStatus.Created = false
	} else if err != nil {
		return err
	} else {
		instance.Status.ServiceStatus.Created = true
	}

	discoveryServerPodQuery := &corev1.Pod{}
	err = r.Get(ctx, *instance.GetDiscoveryServerPodMetadata(), discoveryServerPodQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.PodStatus.Created = false
	} else if err != nil {
		return err
	} else {
		instance.Status.PodStatus.Created = true
		instance.Status.PodStatus.IP = discoveryServerPodQuery.Status.PodIP
		instance.Status.PodStatus.Phase = discoveryServerPodQuery.Status.Phase
	}

	discoveryServerConfigMapQuery := &corev1.ConfigMap{}
	err = r.Get(ctx, *instance.GetDiscoveryServerConfigMapMetadata(), discoveryServerConfigMapQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.ConfigMapStatus.Created = false
	} else if err != nil {
		return err
	} else {
		instance.Status.ConfigMapStatus.Created = true
	}

	return nil
}
