package discovery_server

import (
	"context"
	"net"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	robotErr "github.com/robolaunch/robot-operator/internal/error"
	"github.com/robolaunch/robot-operator/internal/resources"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
)

func (r *DiscoveryServerReconciler) reconcileCheckOwnedResources(ctx context.Context, instance *robotv1alpha1.DiscoveryServer) error {

	if instance.Spec.Attached {

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

	}

	discoveryServerConfigMapQuery := &corev1.ConfigMap{}
	err := r.Get(ctx, *instance.GetDiscoveryServerConfigMapMetadata(), discoveryServerConfigMapQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.ConfigMapStatus.Created = false
	} else if err != nil {
		return err
	} else {
		instance.Status.ConfigMapStatus.Created = true
	}

	return nil
}

func (r *DiscoveryServerReconciler) reconcileUpdateConnectionInfo(ctx context.Context, instance *robotv1alpha1.DiscoveryServer) error {

	if instance.Status.Phase == robotv1alpha1.DiscoveryServerPhaseReady {
		dnsName := resources.GetDiscoveryServerDNS(*instance)

		ips, err := net.LookupIP(dnsName)
		if err != nil {
			return &robotErr.CannotResolveDiscoveryServerError{
				ResourceKind:      instance.Kind,
				ResourceName:      instance.Name,
				ResourceNamespace: instance.Namespace,
			}
		} else if len(ips) == 0 {
			return &robotErr.CannotResolveDiscoveryServerError{
				ResourceKind:      instance.Kind,
				ResourceName:      instance.Name,
				ResourceNamespace: instance.Namespace,
			}
		}

		instance.Status.ConnectionInfo.IP = ips[0].String()
		instance.Status.ConnectionInfo.ConfigMapName = instance.GetDiscoveryServerConfigMapMetadata().Name
	} else {
		instance.Status.ConnectionInfo = robotv1alpha1.ConnectionInfo{}
	}

	return nil
}
