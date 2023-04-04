package discovery_server

import (
	"context"
	"net"

	robotErr "github.com/robolaunch/robot-operator/internal/error"
	"github.com/robolaunch/robot-operator/internal/handle"
	"github.com/robolaunch/robot-operator/internal/reference"
	"github.com/robolaunch/robot-operator/internal/resources"
	mcsv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/external/apis/mcsv1alpha1/v1alpha1"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	"sigs.k8s.io/controller-runtime/pkg/client"
)

func (r *DiscoveryServerReconciler) reconcileUpdateConnectionInfo(ctx context.Context, instance *robotv1alpha1.DiscoveryServer) error {

	if instance.Spec.Type == robotv1alpha1.DiscoveryServerInstanceTypeClient || instance.Status.PodStatus.Resource.Phase == string(corev1.PodRunning) {
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

func (r *DiscoveryServerReconciler) reconcileCheckPod(ctx context.Context, instance *robotv1alpha1.DiscoveryServer) error {

	discoveryServerPodQuery := &corev1.Pod{}
	err := r.Get(ctx, *instance.GetDiscoveryServerPodMetadata(), discoveryServerPodQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.PodStatus.Resource.Created = false
	} else if err != nil {
		return err
	} else {

		err := handle.HandlePod(ctx, r, *discoveryServerPodQuery)
		if err != nil {
			return err
		}

		if discoveryServerPodQuery.Spec.Hostname != instance.Spec.Hostname || discoveryServerPodQuery.Spec.Subdomain != instance.GetDiscoveryServerServiceMetadata().Name {
			err = r.Delete(ctx, discoveryServerPodQuery)
			if err != nil {
				return err
			}

			instance.Status.PodStatus.IP = ""
			instance.Status.PodStatus.Resource.Phase = ""
			instance.Status.Phase = ""

		} else {

			instance.Status.PodStatus.Resource.Created = true
			reference.SetReference(&instance.Status.PodStatus.Resource.Reference, discoveryServerPodQuery.TypeMeta, discoveryServerPodQuery.ObjectMeta)
			instance.Status.PodStatus.IP = discoveryServerPodQuery.Status.PodIP
			instance.Status.PodStatus.Resource.Phase = string(discoveryServerPodQuery.Status.Phase)

		}

	}

	return nil
}

func (r *DiscoveryServerReconciler) reconcileCheckService(ctx context.Context, instance *robotv1alpha1.DiscoveryServer) error {

	err := r.reconcileCleanupOwnedServices(ctx, instance)
	if err != nil {
		return err
	}

	discoveryServerServiceQuery := &corev1.Service{}
	err = r.Get(ctx, *instance.GetDiscoveryServerServiceMetadata(), discoveryServerServiceQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.ServiceStatus.Created = false
	} else if err != nil {
		return err
	} else {
		instance.Status.ServiceStatus.Created = true
		reference.SetReference(&instance.Status.ServiceStatus.Reference, discoveryServerServiceQuery.TypeMeta, discoveryServerServiceQuery.ObjectMeta)
	}

	return nil
}

func (r *DiscoveryServerReconciler) reconcileCheckServiceExport(ctx context.Context, instance *robotv1alpha1.DiscoveryServer) error {

	discoveryServerServiceExportQuery := &mcsv1alpha1.ServiceExport{}
	err := r.Get(ctx, *instance.GetDiscoveryServerServiceMetadata(), discoveryServerServiceExportQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.ServiceExportStatus.Created = false
	} else if err != nil {
		return err
	} else {
		instance.Status.ServiceExportStatus.Created = true
		reference.SetReference(&instance.Status.ServiceExportStatus.Reference, discoveryServerServiceExportQuery.TypeMeta, discoveryServerServiceExportQuery.ObjectMeta)
	}

	return nil
}

func (r *DiscoveryServerReconciler) reconcileCleanupOwnedServices(ctx context.Context, instance *robotv1alpha1.DiscoveryServer) error {

	// service cleanup
	services := &corev1.ServiceList{}
	err := r.List(ctx, services, &client.ListOptions{Namespace: instance.Namespace})
	if err != nil {
		return err
	}

	for _, svc := range services.Items {
		for _, owner := range svc.OwnerReferences {
			if owner.Name == instance.Name && svc.Name != instance.GetDiscoveryServerServiceMetadata().Name {
				err := r.Delete(ctx, &svc)
				if err != nil {
					return err
				}
			}
		}
	}

	return nil
}

func (r *DiscoveryServerReconciler) reconcileCheckConfigMap(ctx context.Context, instance *robotv1alpha1.DiscoveryServer) error {

	discoveryServerConfigMapQuery := &corev1.ConfigMap{}
	err := r.Get(ctx, *instance.GetDiscoveryServerConfigMapMetadata(), discoveryServerConfigMapQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.ConfigMapStatus.Created = false
	} else if err != nil {
		return err
	} else {

		if configuredIP, ok := discoveryServerConfigMapQuery.Labels["configuredIP"]; !ok {
			err := r.Delete(ctx, discoveryServerConfigMapQuery)
			if err != nil {
				return err
			}
		} else {
			if configuredIP != instance.Status.ConnectionInfo.IP {
				err := r.Delete(ctx, discoveryServerConfigMapQuery)
				if err != nil {
					return err
				}
			}
		}

		instance.Status.ConfigMapStatus.Created = true
		reference.SetReference(&instance.Status.ConfigMapStatus.Reference, discoveryServerConfigMapQuery.TypeMeta, discoveryServerConfigMapQuery.ObjectMeta)

	}

	return nil
}
