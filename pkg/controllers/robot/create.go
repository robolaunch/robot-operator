package robot

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	resources "github.com/robolaunch/robot-operator/pkg/controllers/robot/spawn"
	"k8s.io/apimachinery/pkg/types"
	ctrl "sigs.k8s.io/controller-runtime"
)

func (r *RobotReconciler) createPVC(ctx context.Context, instance *robotv1alpha1.Robot, pvcNamespacedName *types.NamespacedName) error {

	pvc := resources.GetPersistentVolumeClaim(instance, pvcNamespacedName)

	err := ctrl.SetControllerReference(instance, pvc, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, pvc)
	if err != nil {
		return err
	}

	logger.Info("STATUS: PVC " + pvc.Name + " is created.")
	return nil
}

func (r *RobotReconciler) createDiscoveryServer(ctx context.Context, instance *robotv1alpha1.Robot, dsNamespacedName *types.NamespacedName) error {

	discoveryServer := resources.GetDiscoveryServer(instance, dsNamespacedName)

	err := ctrl.SetControllerReference(instance, discoveryServer, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, discoveryServer)
	if err != nil {
		return err
	}

	logger.Info("STATUS: Discovery server " + discoveryServer.Name + " is created.")
	return nil
}
