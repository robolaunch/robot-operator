package ros_bridge

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	"github.com/robolaunch/robot-operator/pkg/controllers/robot/ros_bridge/spawn"
	"k8s.io/apimachinery/pkg/types"
	ctrl "sigs.k8s.io/controller-runtime"
)

func (r *ROSBridgeReconciler) createService(ctx context.Context, instance *robotv1alpha1.ROSBridge, svcNamespacedName *types.NamespacedName) error {

	svc := spawn.GetBridgeService(instance, instance.GetBridgeServiceMetadata())

	err := ctrl.SetControllerReference(instance, svc, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, svc)
	if err != nil {
		return err
	}

	logger.Info("STATUS: Service " + svc.Name + " is created.")
	return nil
}

func (r *ROSBridgeReconciler) createPod(ctx context.Context, instance *robotv1alpha1.ROSBridge, podNamespacedName *types.NamespacedName) error {

	robot, err := r.reconcileGetOwner(ctx, instance)
	if err != nil {
		return err
	}

	pod := spawn.GetBridgePod(instance, instance.GetBridgePodMetadata(), *robot)

	err = ctrl.SetControllerReference(instance, pod, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, pod)
	if err != nil {
		return err
	}

	logger.Info("STATUS: Pod " + pod.Name + " is created.")
	return nil
}
