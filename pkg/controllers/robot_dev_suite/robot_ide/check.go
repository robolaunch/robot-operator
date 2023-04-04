package robot_ide

import (
	"context"

	"github.com/robolaunch/robot-operator/internal/handle"
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
			instance.Status.ServiceStatus.Created = false
		} else {
			return err
		}
	} else {
		instance.Status.ServiceStatus.Created = true
	}

	return nil
}

func (r *RobotIDEReconciler) reconcileCheckPod(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {

	podQuery := &corev1.Pod{}
	err := r.Get(ctx, *instance.GetRobotIDEPodMetadata(), podQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.PodStatus = robotv1alpha1.DetailedOwnedPodStatus{}
		} else {
			return err
		}
	} else {

		err = handle.HandlePod(ctx, r, *podQuery)
		if err != nil {
			return err
		}

		instance.Status.PodStatus.Created = true
		instance.Status.PodStatus.Phase = podQuery.Status.Phase
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
				instance.Status.IngressStatus.Created = false
			} else {
				return err
			}
		} else {
			instance.Status.IngressStatus.Created = true
		}
	}

	return nil
}
