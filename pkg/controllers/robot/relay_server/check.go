package relay_server

import (
	"context"

	"github.com/robolaunch/robot-operator/internal/handle"
	"github.com/robolaunch/robot-operator/internal/reference"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	networkingv1 "k8s.io/api/networking/v1"
	"k8s.io/apimachinery/pkg/api/errors"
)

func (r *RelayServerReconciler) reconcileCheckPod(ctx context.Context, instance *robotv1alpha1.RelayServer) error {

	podQuery := &corev1.Pod{}
	err := r.Get(ctx, *instance.GetRelayServerPodMetadata(), podQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.PodStatus = robotv1alpha1.OwnedResourceStatus{}
		} else {
			return err
		}
	} else {

		err = handle.HandlePod(ctx, r, *podQuery)
		if err != nil {
			return err
		}

		instance.Status.PodStatus.Created = true
		reference.SetReference(&instance.Status.PodStatus.Reference, podQuery.TypeMeta, podQuery.ObjectMeta)
		instance.Status.PodStatus.Phase = string(podQuery.Status.Phase)
	}

	return nil
}

func (r *RelayServerReconciler) reconcileCheckService(ctx context.Context, instance *robotv1alpha1.RelayServer) error {

	serviceQuery := &corev1.Service{}
	err := r.Get(ctx, *instance.GetRelayServerServiceMetadata(), serviceQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.ServiceStatus = robotv1alpha1.OwnedServiceStatus{}
		} else {
			return err
		}
	} else {

		instance.Status.ServiceStatus.Resource.Created = true
		reference.SetReference(&instance.Status.ServiceStatus.Resource.Reference, serviceQuery.TypeMeta, serviceQuery.ObjectMeta)
		instance.Status.ServiceStatus.URLs = map[string]string{}
		instance.Status.ServiceStatus.URLs["relay-server"] = robotv1alpha1.GetRelayServerServiceDNS(*instance, "https://", "/relay/")
	}

	return nil
}

func (r *RelayServerReconciler) reconcileCheckIngress(ctx context.Context, instance *robotv1alpha1.RelayServer) error {

	ingressQuery := &networkingv1.Ingress{}
	err := r.Get(ctx, *instance.GetRelayServerIngressMetadata(), ingressQuery)
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

	return nil
}
