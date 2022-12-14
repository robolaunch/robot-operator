package robot_vdi

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	"github.com/robolaunch/robot-operator/internal/resources"
	ctrl "sigs.k8s.io/controller-runtime"
)

func (r *RobotVDIReconciler) reconcileCreatePVC(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	vdiPVC := resources.GetRobotVDIPVC(instance, instance.GetRobotVDIPVCMetadata(), *robot)

	err = ctrl.SetControllerReference(instance, vdiPVC, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, vdiPVC)
	if err != nil {
		return err
	}

	logger.Info("STATUS: VDI PVC is created.")

	return nil
}

func (r *RobotVDIReconciler) reconcileCreateServiceTCP(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	vdiService := resources.GetRobotVDIServiceTCP(instance, instance.GetRobotVDIServiceTCPMetadata())

	err := ctrl.SetControllerReference(instance, vdiService, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, vdiService)
	if err != nil {
		return err
	}

	logger.Info("STATUS: VDI TCP service is created.")

	return nil
}

func (r *RobotVDIReconciler) reconcileCreateServiceUDP(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	vdiService := resources.GetRobotVDIServiceUDP(instance, instance.GetRobotVDIServiceUDPMetadata())

	err := ctrl.SetControllerReference(instance, vdiService, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, vdiService)
	if err != nil {
		return err
	}

	logger.Info("STATUS: VDI UDP service is created.")

	return nil
}

func (r *RobotVDIReconciler) reconcileCreatePod(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	vdiService := resources.GetRobotVDIPod(instance, instance.GetRobotVDIPodMetadata(), *robot)

	err = ctrl.SetControllerReference(instance, vdiService, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, vdiService)
	if err != nil {
		return err
	}

	logger.Info("STATUS: VDI TCP service is created.")

	return nil
}

func (r *RobotVDIReconciler) reconcileCreateIngress(ctx context.Context, instance *robotv1alpha1.RobotVDI) error {

	robot, err := r.reconcileGetTargetRobot(ctx, instance)
	if err != nil {
		return err
	}

	vdiIngress := resources.GetRobotVDIIngress(instance, instance.GetRobotVDIPodMetadata(), *robot)

	err = ctrl.SetControllerReference(instance, vdiIngress, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, vdiIngress)
	if err != nil {
		return err
	}

	logger.Info("STATUS: VDI ingress is created.")

	return nil
}
