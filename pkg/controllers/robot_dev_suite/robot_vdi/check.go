package robot_vdi

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	networkingv1 "k8s.io/api/networking/v1"
	"k8s.io/apimachinery/pkg/api/errors"
)

func (r *RobotVDIReconciler) reconcileCheckPVC(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	if !instance.Spec.MainDisplay {

	}

	return nil
}

func (r *RobotVDIReconciler) reconcileCheckServices(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	serviceTCPQuery := &corev1.Service{}
	err := r.Get(ctx, *instance.GetRobotVDIServiceTCPMetadata(), serviceTCPQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.ServiceTCPStatus.Created = false
		} else {
			return err
		}
	} else {
		instance.Status.ServiceTCPStatus.Created = true
	}

	serviceUDPQuery := &corev1.Service{}
	err = r.Get(ctx, *instance.GetRobotVDIServiceUDPMetadata(), serviceUDPQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.ServiceUDPStatus.Created = false
		} else {
			return err
		}
	} else {
		instance.Status.ServiceUDPStatus.Created = true
	}

	return nil
}

func (r *RobotVDIReconciler) reconcileCheckPod(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	vdiPodQuery := &corev1.Pod{}
	err := r.Get(ctx, *instance.GetRobotVDIPodMetadata(), vdiPodQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.PodStatus = robotv1alpha1.RobotVDIPodStatus{}
		} else {
			return err
		}
	} else {
		instance.Status.PodStatus.Created = true
		instance.Status.PodStatus.Phase = vdiPodQuery.Status.Phase
		instance.Status.PodStatus.IP = vdiPodQuery.Status.PodIP
	}

	return nil
}

func (r *RobotVDIReconciler) reconcileCheckIngress(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	if instance.Spec.Ingress {
		ingressQuery := &networkingv1.Ingress{}
		err := r.Get(ctx, *instance.GetRobotVDIIngressMetadata(), ingressQuery)
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
