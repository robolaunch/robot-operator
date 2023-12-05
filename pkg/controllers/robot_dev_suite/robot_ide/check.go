package robot_ide

import (
	"context"
	"strconv"

	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/handle"
	"github.com/robolaunch/robot-operator/internal/reference"
	mcsv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/external/apis/mcsv1alpha1/v1alpha1"
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
			instance.Status.ServiceStatus = robotv1alpha1.OwnedServiceStatus{}
		} else {
			return err
		}
	} else {
		robot, err := r.reconcileGetTargetRobot(ctx, instance)
		if err != nil {
			return err
		}

		instance.Status.ServiceStatus.Resource.Created = true
		reference.SetReference(&instance.Status.ServiceStatus.Resource.Reference, serviceQuery.TypeMeta, serviceQuery.ObjectMeta)
		if instance.Spec.Ingress {
			instance.Status.ServiceStatus.URLs = map[string]string{}
			instance.Status.ServiceStatus.URLs["code-server"] = robotv1alpha1.GetRobotServiceDNS(*robot, "https://", "/ide/")
			instance.Status.ServiceStatus.URLs["code-server-file-browser"] = robotv1alpha1.GetRobotServiceDNS(*robot, "https://", "/file-browser/ide/")
		} else if instance.Spec.ServiceType == corev1.ServiceTypeNodePort {
			instance.Status.ServiceStatus.URLs = map[string]string{}
			instance.Status.ServiceStatus.URLs["code-server"] = robotv1alpha1.GetRobotServiceDNSWithNodePort(*robot, "http://", strconv.Itoa(int(serviceQuery.Spec.Ports[0].NodePort)))
			instance.Status.ServiceStatus.URLs["code-server-file-browser"] = robotv1alpha1.GetRobotServiceDNSWithNodePort(*robot, "http://", strconv.Itoa(int(serviceQuery.Spec.Ports[1].NodePort)))
		}
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

func (r *RobotIDEReconciler) reconcileCheckServiceExport(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {

	serviceExportQuery := &mcsv1alpha1.ServiceExport{}
	err := r.Get(ctx, *instance.GetRobotIDEServiceExportMetadata(), serviceExportQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.ServiceExportStatus = robotv1alpha1.OwnedResourceStatus{}
		} else {
			return err
		}
	} else {
		instance.Status.ServiceExportStatus.Created = true
		reference.SetReference(&instance.Status.ServiceExportStatus.Reference, serviceExportQuery.TypeMeta, serviceExportQuery.ObjectMeta)
	}

	return nil
}

func (r *RobotIDEReconciler) reconcileCheckCustomService(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	if config, ok := robot.Spec.AdditionalConfigs[internal.IDE_CUSTOM_PORT_RANGE_KEY]; ok && config.ConfigType == robotv1alpha1.AdditionalConfigTypeOperator {
		customSvcQuery := &corev1.Service{}
		err := r.Get(ctx, *instance.GetRobotIDECustomServiceMetadata(), customSvcQuery)
		if err != nil {
			if errors.IsNotFound(err) {
				instance.Status.CustomPortServiceStatus = robotv1alpha1.OwnedServiceStatus{}
			} else {
				return err
			}
		} else {
			instance.Status.CustomPortServiceStatus.Resource.Created = true
			reference.SetReference(&instance.Status.CustomPortServiceStatus.Resource.Reference, customSvcQuery.TypeMeta, customSvcQuery.ObjectMeta)
		}
	}

	return nil
}

func (r *RobotIDEReconciler) reconcileCheckCustomIngress(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {

	if instance.Spec.Ingress {

		robot, err := r.reconcileGetTargetRobot(ctx, instance)
		if err != nil {
			return err
		}

		if config, ok := robot.Spec.AdditionalConfigs[internal.IDE_CUSTOM_PORT_RANGE_KEY]; ok && config.ConfigType == robotv1alpha1.AdditionalConfigTypeOperator {
			customIngressQuery := &networkingv1.Ingress{}
			err := r.Get(ctx, *instance.GetRobotIDECustomIngressMetadata(), customIngressQuery)
			if err != nil {
				if errors.IsNotFound(err) {
					instance.Status.CustomPortIngressStatus = robotv1alpha1.OwnedResourceStatus{}
				} else {
					return err
				}
			} else {
				instance.Status.CustomPortIngressStatus.Created = true
				reference.SetReference(&instance.Status.CustomPortIngressStatus.Reference, customIngressQuery.TypeMeta, customIngressQuery.ObjectMeta)
			}
		}

	}

	return nil
}

func (r *RobotIDEReconciler) reconcileCheckConfigMap(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {

	cmQuery := &corev1.ConfigMap{}
	err := r.Get(ctx, *instance.GetRobotIDEConfigMapMetadata(), cmQuery)
	if err != nil {
		if errors.IsNotFound(err) {
			instance.Status.ConfigMapStatus = robotv1alpha1.OwnedResourceStatus{}
		} else {
			return err
		}
	} else {
		instance.Status.ConfigMapStatus.Created = true
		reference.SetReference(&instance.Status.IngressStatus.Reference, cmQuery.TypeMeta, cmQuery.ObjectMeta)
	}

	return nil
}
