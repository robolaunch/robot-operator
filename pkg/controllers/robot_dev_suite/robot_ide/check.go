package robot_ide

import (
	"context"

	"github.com/robolaunch/robot-operator/internal/handle"
	"github.com/robolaunch/robot-operator/internal/reference"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	networkingv1 "k8s.io/api/networking/v1"
	"k8s.io/apimachinery/pkg/api/errors"
)

func (r *RobotIDEReconciler) reconcileCheckService(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {

	serviceQuery := &corev1.Service{}
	err := r.Get(ctx, *instance.GetRobotIDEServiceMetadata(), serviceQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.ServiceStatus = robotv1alpha1.OwnedResourceStatus{}
		} else {
			return err
		}
	} else {
		instance.Status.ServiceStatus.Created = true
		reference.SetReference(&instance.Status.ServiceStatus.Reference, serviceQuery.TypeMeta, serviceQuery.ObjectMeta)
	}

	return nil
}

func (r *RobotIDEReconciler) reconcileCheckPod(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {

	podQuery := &corev1.Pod{}
	err := r.Get(ctx, *instance.GetRobotIDEPodMetadata(), podQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.PodStatus = robotv1alpha1.OwnedPodStatus{}
		} else {
			return err
		}
	} else {

		err = handle.HandlePod(ctx, r, *podQuery)
		if err != nil {
			return err
		}

		instance.Status.PodStatus.Resource.Created = true
		reference.SetReference(&instance.Status.PodStatus.Resource.Reference, podQuery.TypeMeta, podQuery.ObjectMeta)
		instance.Status.PodStatus.Resource.Phase = string(podQuery.Status.Phase)
		instance.Status.PodStatus.IP = podQuery.Status.PodIP
	}

	return nil
}

func (r *RobotIDEReconciler) reconcileCheckIngress(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {

	if instance.Spec.Ingress {
		ingressQuery := &networkingv1.Ingress{}
		err := r.Get(ctx, *instance.GetRobotIDEIngressMetadata(), ingressQuery)
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
