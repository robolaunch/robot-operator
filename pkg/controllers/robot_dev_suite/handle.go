package robot_dev_suite

import (
	"context"

	robotErr "github.com/robolaunch/robot-operator/internal/error"
	"github.com/robolaunch/robot-operator/internal/label"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
)

func (r *RobotDevSuiteReconciler) reconcileHandleRobotVDI(ctx context.Context, instance *robotv1alpha1.RobotDevSuite) error {

	if instance.Spec.VDIEnabled {
		if !instance.Status.RobotVDIStatus.Resource.Created {
			instance.Status.Phase = robotv1alpha1.RobotDevSuitePhaseCreatingRobotVDI
			err := r.reconcileCreateRobotVDI(ctx, instance)
			if err != nil {
				return err
			}
			instance.Status.RobotVDIStatus.Resource.Created = true

			return &robotErr.CreatingResourceError{
				ResourceKind:      "RobotVDI",
				ResourceName:      instance.GetRobotVDIMetadata().Name,
				ResourceNamespace: instance.GetRobotVDIMetadata().Namespace,
			}
		}

		if instance.Status.RobotVDIStatus.Resource.Phase != string(robotv1alpha1.RobotVDIPhaseRunning) {
			return &robotErr.WaitingForResourceError{
				ResourceKind:      "RobotVDI",
				ResourceName:      instance.GetRobotVDIMetadata().Name,
				ResourceNamespace: instance.GetRobotVDIMetadata().Namespace,
			}
		}

	}

	return nil
}

func (r *RobotDevSuiteReconciler) reconcileHandleRobotIDE(ctx context.Context, instance *robotv1alpha1.RobotDevSuite) error {

	if instance.Spec.IDEEnabled {
		if !instance.Status.RobotIDEStatus.Resource.Created {
			instance.Status.Phase = robotv1alpha1.RobotDevSuitePhaseCreatingRobotIDE
			err := r.reconcileCreateRobotIDE(ctx, instance)
			if err != nil {
				return err
			}
			instance.Status.RobotIDEStatus.Resource.Created = true

			return &robotErr.CreatingResourceError{
				ResourceKind:      "RobotIDE",
				ResourceName:      instance.GetRobotIDEMetadata().Name,
				ResourceNamespace: instance.GetRobotIDEMetadata().Namespace,
			}
		}

		if instance.Status.RobotIDEStatus.Resource.Phase != string(robotv1alpha1.RobotIDEPhaseRunning) {
			return &robotErr.WaitingForResourceError{
				ResourceKind:      "RobotIDE",
				ResourceName:      instance.GetRobotIDEMetadata().Name,
				ResourceNamespace: instance.GetRobotIDEMetadata().Namespace,
			}
		}
	}

	return nil
}

func (r *RobotDevSuiteReconciler) reconcileHandleRemoteIDE(ctx context.Context, instance *robotv1alpha1.RobotDevSuite) error {

	if instance.Spec.RemoteIDEEnabled && label.GetInstanceType(instance) == label.InstanceTypeCloudInstance {

		if !instance.Status.RemoteIDERelayServerStatus.Resource.Created {
			instance.Status.Phase = robotv1alpha1.RobotDevSuitePhaseCreatingRelayServerForRemoteIDE
			err := r.reconcileCreateRemoteIDERelayServer(ctx, instance)
			if err != nil {
				return err
			}
			instance.Status.RemoteIDERelayServerStatus.Resource.Created = true

			return &robotErr.CreatingResourceError{
				ResourceKind:      "RelayServer",
				ResourceName:      instance.GetRemoteIDERelayServerMetadata().Name,
				ResourceNamespace: instance.GetRemoteIDERelayServerMetadata().Namespace,
			}
		}

		if instance.Status.RemoteIDERelayServerStatus.Resource.Phase != string(robotv1alpha1.RelayServerPhaseReady) {
			return &robotErr.WaitingForResourceError{
				ResourceKind:      "RelayServer",
				ResourceName:      instance.GetRemoteIDERelayServerMetadata().Name,
				ResourceNamespace: instance.GetRemoteIDERelayServerMetadata().Namespace,
			}
		}

	}

	return nil
}
