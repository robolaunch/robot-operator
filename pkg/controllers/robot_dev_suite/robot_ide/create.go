package robot_ide

import (
	"context"

	"github.com/robolaunch/robot-operator/internal/label"
	"github.com/robolaunch/robot-operator/internal/resources"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	ctrl "sigs.k8s.io/controller-runtime"
)

func (r *RobotIDEReconciler) reconcileCreateService(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {

	ideService := resources.GetRobotIDEService(instance, instance.GetRobotIDEServiceMetadata())

	err := ctrl.SetControllerReference(instance, ideService, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, ideService)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: IDE service is created.")

	return nil
}

func (r *RobotIDEReconciler) reconcileCreatePod(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	robotVDI := &robotv1alpha1.RobotVDI{}
	if label.GetTargetRobotVDI(instance) != "" {
		robotVDI, err = r.reconcileGetTargetRobotVDI(ctx, instance)
		if err != nil {
			return err
		}
	}

	activeNode, err := r.reconcileCheckNode(ctx, robot)
	if err != nil {
		return err
	}

	cm := corev1.ConfigMap{}
	err = r.Get(ctx, *instance.GetRobotIDEConfigMapMetadata(), &cm)
	if err != nil {
		return err
	}

	idePod := resources.GetRobotIDEPod(instance, instance.GetRobotIDEPodMetadata(), *robot, *robotVDI, *activeNode, cm)

	err = ctrl.SetControllerReference(instance, idePod, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, idePod)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: IDE pod is created.")

	return nil
}

func (r *RobotIDEReconciler) reconcileCreateIngress(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	ideIngress := resources.GetRobotIDEIngress(instance, instance.GetRobotIDEIngressMetadata(), *robot)

	err = ctrl.SetControllerReference(instance, ideIngress, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, ideIngress)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: IDE ingress is created.")

	return nil
}

func (r *RobotIDEReconciler) reconcileCreateServiceExport(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {

	ideServiceExport := resources.GetRobotIDEServiceExport(instance, instance.GetRobotIDEServiceExportMetadata())

	err := ctrl.SetControllerReference(instance, ideServiceExport, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, ideServiceExport)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: IDE service export is created.")

	return nil
}

func (r *RobotIDEReconciler) reconcileCreateCustomService(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	ideService := resources.GetRobotIDECustomService(instance, instance.GetRobotIDECustomServiceMetadata(), *robot)

	err = ctrl.SetControllerReference(instance, ideService, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, ideService)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: IDE custom service is created.")

	return nil
}

func (r *RobotIDEReconciler) reconcileCreateCustomIngress(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	ideIngress := resources.GetRobotIDECustomIngress(instance, instance.GetRobotIDECustomIngressMetadata(), *robot)

	err = ctrl.SetControllerReference(instance, ideIngress, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, ideIngress)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: IDE custom ingress is created.")

	return nil
}

func (r *RobotIDEReconciler) reconcileCreateConfigMap(ctx context.Context, instance *robotv1alpha1.RobotIDE) error {

	ideCm := resources.GetRobotIDEConfigMap(instance, instance.GetRobotIDEConfigMapMetadata())

	err := ctrl.SetControllerReference(instance, ideCm, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, ideCm)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: IDE config map is created.")

	return nil
}
