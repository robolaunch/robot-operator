package robot

import (
	"context"

	robotErr "github.com/robolaunch/robot-operator/internal/error"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
)

func (r *RobotReconciler) reconcileHandlePVCs(ctx context.Context, instance *robotv1alpha1.Robot) error {

	if !(instance.Status.VolumeStatuses.Var.Created &&
		instance.Status.VolumeStatuses.Opt.Created &&
		instance.Status.VolumeStatuses.Etc.Created &&
		instance.Status.VolumeStatuses.Usr.Created &&
		instance.Status.VolumeStatuses.Workspace.Created) {
		instance.Status.Phase = robotv1alpha1.RobotPhaseCreatingEnvironment

		if !instance.Status.VolumeStatuses.Var.Created {
			err := r.createPVC(ctx, instance, instance.GetPVCVarMetadata())
			if err != nil {
				return err
			}
			instance.Status.VolumeStatuses.Var.Created = true
		}

		if !instance.Status.VolumeStatuses.Opt.Created {
			err := r.createPVC(ctx, instance, instance.GetPVCOptMetadata())
			if err != nil {
				return err
			}
			instance.Status.VolumeStatuses.Opt.Created = true
		}

		if !instance.Status.VolumeStatuses.Etc.Created {
			err := r.createPVC(ctx, instance, instance.GetPVCEtcMetadata())
			if err != nil {
				return err
			}
			instance.Status.VolumeStatuses.Etc.Created = true
		}

		if !instance.Status.VolumeStatuses.Usr.Created {
			err := r.createPVC(ctx, instance, instance.GetPVCUsrMetadata())
			if err != nil {
				return err
			}
			instance.Status.VolumeStatuses.Usr.Created = true
		}

		if !instance.Status.VolumeStatuses.Workspace.Created {
			err := r.createPVC(ctx, instance, instance.GetPVCWorkspaceMetadata())
			if err != nil {
				return err
			}
			instance.Status.VolumeStatuses.Workspace.Created = true
		}

		return &robotErr.CreatingResourceError{
			ResourceKind:      "PersistentVolumeClaim",
			ResourceNamespace: instance.Namespace,
		}

	}

	return nil
}

func (r *RobotReconciler) reconcileHandleDiscoveryServer(ctx context.Context, instance *robotv1alpha1.Robot) error {

	if !instance.Status.DiscoveryServerStatus.Resource.Created {
		instance.Status.Phase = robotv1alpha1.RobotPhaseCreatingDiscoveryServer
		err := r.createDiscoveryServer(ctx, instance, instance.GetDiscoveryServerMetadata())
		if err != nil {
			return err
		}
		instance.Status.DiscoveryServerStatus.Resource.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "DiscoveryServer",
			ResourceName:      instance.GetDiscoveryServerMetadata().Name,
			ResourceNamespace: instance.GetDiscoveryServerMetadata().Namespace,
		}
	}

	return nil
}

func (r *RobotReconciler) reconcileHandleLoaderJob(ctx context.Context, instance *robotv1alpha1.Robot) error {
	if !instance.Status.LoaderJobStatus.Created {
		instance.Status.Phase = robotv1alpha1.RobotPhaseConfiguringEnvironment
		err := r.createJob(ctx, instance, instance.GetLoaderJobMetadata())
		if err != nil {
			return err
		}
		instance.Status.LoaderJobStatus.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "Job",
			ResourceName:      instance.GetLoaderJobMetadata().Name,
			ResourceNamespace: instance.GetLoaderJobMetadata().Namespace,
		}
	}

	if instance.Status.LoaderJobStatus.Phase != string(robotv1alpha1.JobSucceeded) {

		switch instance.Status.LoaderJobStatus.Phase {
		case string(robotv1alpha1.JobActive):
			instance.Status.Phase = robotv1alpha1.RobotPhaseConfiguringEnvironment
		case string(robotv1alpha1.JobFailed):
			// TODO: add reason
			instance.Status.Phase = robotv1alpha1.RobotPhaseFailed
		}

		return &robotErr.WaitingForResourceError{
			ResourceKind:      "Job",
			ResourceName:      instance.GetLoaderJobMetadata().Name,
			ResourceNamespace: instance.GetLoaderJobMetadata().Namespace,
		}

	}

	return nil
}

func (r *RobotReconciler) reconcileHandleROSBridge(ctx context.Context, instance *robotv1alpha1.Robot) error {
	if instance.Spec.ROSBridgeTemplate.ROS.Enabled || instance.Spec.ROSBridgeTemplate.ROS2.Enabled {
		if !instance.Status.ROSBridgeStatus.Resource.Created {
			instance.Status.Phase = robotv1alpha1.RobotPhaseCreatingBridge
			err := r.createROSBridge(ctx, instance, instance.GetROSBridgeMetadata())
			if err != nil {
				return err
			}
			instance.Status.ROSBridgeStatus.Resource.Created = true

			return &robotErr.CreatingResourceError{
				ResourceKind:      "ROSBridge",
				ResourceName:      instance.GetROSBridgeMetadata().Name,
				ResourceNamespace: instance.GetROSBridgeMetadata().Namespace,
			}
		}
		if instance.Status.ROSBridgeStatus.Status.Phase != robotv1alpha1.BridgePhaseReady {
			return &robotErr.WaitingForResourceError{
				ResourceKind:      "ROSBridge",
				ResourceName:      instance.GetROSBridgeMetadata().Name,
				ResourceNamespace: instance.GetROSBridgeMetadata().Namespace,
			}
		}
	}

	return nil
}

func (r *RobotReconciler) reconcileHandleRobotDevSuite(ctx context.Context, instance *robotv1alpha1.Robot) error {
	if instance.Spec.RobotDevSuiteTemplate.IDEEnabled || instance.Spec.RobotDevSuiteTemplate.VDIEnabled || instance.Spec.RobotDevSuiteTemplate.RemoteIDEEnabled {
		if !instance.Status.RobotDevSuiteStatus.Resource.Created {
			instance.Status.Phase = robotv1alpha1.RobotPhaseCreatingDevelopmentSuite
			err := r.createRobotDevSuite(ctx, instance, instance.GetRobotDevSuiteMetadata())
			if err != nil {
				return err
			}
			instance.Status.RobotDevSuiteStatus.Resource.Created = true

			return &robotErr.CreatingResourceError{
				ResourceKind:      "RobotDevSuite",
				ResourceName:      instance.GetRobotDevSuiteMetadata().Name,
				ResourceNamespace: instance.GetRobotDevSuiteMetadata().Namespace,
			}
		}
	}
	return nil
}

func (r *RobotReconciler) reconcileHandleWorkspaceManager(ctx context.Context, instance *robotv1alpha1.Robot) error {
	if !instance.Status.WorkspaceManagerStatus.Resource.Created {
		instance.Status.Phase = robotv1alpha1.RobotPhaseConfiguringWorkspaces
		err := r.createWorkspaceManager(ctx, instance, instance.GetWorkspaceManagerMetadata())
		if err != nil {
			return err
		}
		instance.Status.WorkspaceManagerStatus.Resource.Created = true

		return &robotErr.CreatingResourceError{
			ResourceKind:      "WorkspaceManager",
			ResourceName:      instance.GetWorkspaceManagerMetadata().Name,
			ResourceNamespace: instance.GetWorkspaceManagerMetadata().Namespace,
		}
	}

	if instance.Status.WorkspaceManagerStatus.Status.Phase != robotv1alpha1.WorkspaceManagerPhaseReady {

		// LaunchManagers or BuildManagers shouldn't be attached if WorkspaceManager is not ready.
		instance.Status.AttachedBuildObject = robotv1alpha1.AttachedBuildObject{}
		instance.Status.AttachedLaunchObjects = []robotv1alpha1.AttachedLaunchObject{}

		return &robotErr.WaitingForResourceError{
			ResourceKind:      "WorkspaceManager",
			ResourceName:      instance.GetWorkspaceManagerMetadata().Name,
			ResourceNamespace: instance.GetWorkspaceManagerMetadata().Namespace,
		}
	}

	instance.Status.Phase = robotv1alpha1.RobotPhaseEnvironmentReady

	return nil
}

func (r *RobotReconciler) reconcileHandleManagers(ctx context.Context, instance *robotv1alpha1.Robot) error {

	// select attached build object
	err := r.reconcileAttachBuildObject(ctx, instance)
	if err != nil {
		return err
	}

	if instance.Status.AttachedBuildObject.Status.Phase == robotv1alpha1.BuildManagerReady {
		// select attached launch object
		err := r.reconcileAttachLaunchObject(ctx, instance)
		if err != nil {
			return err
		}
	}

	return nil
}
