package robot

import (
	"context"
	"reflect"

	"github.com/robolaunch/robot-operator/internal/reference"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	batchv1 "k8s.io/api/batch/v1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/types"
)

func (r *RobotReconciler) reconcileCheckPVCs(ctx context.Context, instance *robotv1alpha1.Robot) error {

	err := r.reconcileCheckPVC(ctx, instance, *instance.GetPVCVarMetadata(), &instance.Status.VolumeStatuses.Var)
	if err != nil {
		return err
	}

	err = r.reconcileCheckPVC(ctx, instance, *instance.GetPVCOptMetadata(), &instance.Status.VolumeStatuses.Opt)
	if err != nil {
		return err
	}

	err = r.reconcileCheckPVC(ctx, instance, *instance.GetPVCEtcMetadata(), &instance.Status.VolumeStatuses.Etc)
	if err != nil {
		return err
	}

	err = r.reconcileCheckPVC(ctx, instance, *instance.GetPVCUsrMetadata(), &instance.Status.VolumeStatuses.Usr)
	if err != nil {
		return err
	}

	err = r.reconcileCheckPVC(ctx, instance, *instance.GetPVCWorkspaceMetadata(), &instance.Status.VolumeStatuses.Workspace)
	if err != nil {
		return err
	}

	return nil
}

func (r *RobotReconciler) reconcileCheckPVC(ctx context.Context, instance *robotv1alpha1.Robot, pvcNamespacedName types.NamespacedName, pvcStatus *robotv1alpha1.OwnedResourceStatus) error {
	pvcQuery := &corev1.PersistentVolumeClaim{}
	err := r.Get(ctx, pvcNamespacedName, pvcQuery)
	if err != nil && errors.IsNotFound(err) {
		pvcStatus = &robotv1alpha1.OwnedResourceStatus{}
	} else if err != nil {
		return err
	} else {
		pvcStatus.Created = true
		reference.SetReference(&pvcStatus.Reference, pvcQuery.TypeMeta, pvcQuery.ObjectMeta)
	}
	return nil
}

func (r *RobotReconciler) reconcileCheckDiscoveryServer(ctx context.Context, instance *robotv1alpha1.Robot) error {

	discoverServerQuery := &robotv1alpha1.DiscoveryServer{}
	err := r.Get(ctx, *instance.GetDiscoveryServerMetadata(), discoverServerQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.DiscoveryServerStatus = robotv1alpha1.DiscoveryServerInstanceStatus{}
	} else if err != nil {
		return err
	} else {

		if !reflect.DeepEqual(instance.Spec.RobotConfig.DiscoveryServerTemplate, discoverServerQuery.Spec) {
			discoverServerQuery.Spec = instance.Spec.RobotConfig.DiscoveryServerTemplate
			err = r.Update(ctx, discoverServerQuery)
			if err != nil {
				return err
			}
		}

		instance.Status.DiscoveryServerStatus.Resource.Created = true
		reference.SetReference(&instance.Status.DiscoveryServerStatus.Resource.Reference, discoverServerQuery.TypeMeta, discoverServerQuery.ObjectMeta)
		instance.Status.DiscoveryServerStatus.Status = discoverServerQuery.Status
	}

	return nil
}

func (r *RobotReconciler) reconcileCheckLoaderJob(ctx context.Context, instance *robotv1alpha1.Robot) error {

	if instance.Status.Phase != robotv1alpha1.RobotPhaseEnvironmentReady {
		loaderJobQuery := &batchv1.Job{}
		err := r.Get(ctx, *instance.GetLoaderJobMetadata(), loaderJobQuery)
		if err != nil && errors.IsNotFound(err) {
			instance.Status.LoaderJobStatus = robotv1alpha1.OwnedResourceStatus{}
		} else if err != nil {
			return err
		} else {
			reference.SetReference(&instance.Status.LoaderJobStatus.Reference, loaderJobQuery.TypeMeta, loaderJobQuery.ObjectMeta)
			switch 1 {
			case int(loaderJobQuery.Status.Succeeded):
				instance.Status.LoaderJobStatus.Phase = string(robotv1alpha1.JobSucceeded)
			case int(loaderJobQuery.Status.Active):
				instance.Status.LoaderJobStatus.Phase = string(robotv1alpha1.JobActive)
			case int(loaderJobQuery.Status.Failed):
				instance.Status.LoaderJobStatus.Phase = string(robotv1alpha1.JobFailed)
			}
		}
	}

	return nil
}

func (r *RobotReconciler) reconcileCheckROSBridge(ctx context.Context, instance *robotv1alpha1.Robot) error {

	rosBridgeQuery := &robotv1alpha1.ROSBridge{}
	err := r.Get(ctx, *instance.GetROSBridgeMetadata(), rosBridgeQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.ROSBridgeStatus = robotv1alpha1.ROSBridgeInstanceStatus{}
	} else if err != nil {
		return err
	} else {
		if instance.Spec.RobotConfig.ROSBridgeTemplate.ROS2.Enabled {
			if !reflect.DeepEqual(instance.Spec.RobotConfig.ROSBridgeTemplate, rosBridgeQuery.Spec) {
				rosBridgeQuery.Spec = instance.Spec.RobotConfig.ROSBridgeTemplate
				err = r.Update(ctx, rosBridgeQuery)
				if err != nil {
					return err
				}
			}

			instance.Status.ROSBridgeStatus.Resource.Created = true
			reference.SetReference(&instance.Status.ROSBridgeStatus.Resource.Reference, rosBridgeQuery.TypeMeta, rosBridgeQuery.ObjectMeta)
			instance.Status.ROSBridgeStatus.Status = rosBridgeQuery.Status
			instance.Status.ROSBridgeStatus.Connection = rosBridgeQuery.Status.ServiceStatus.URL
		} else {
			err := r.Delete(ctx, rosBridgeQuery)
			if err != nil {
				return err
			}
		}
	}

	return nil
}

func (r *RobotReconciler) reconcileCheckRobotDevSuite(ctx context.Context, instance *robotv1alpha1.Robot) error {

	robotDevSuiteQuery := &robotv1alpha1.RobotDevSuite{}
	err := r.Get(ctx, *instance.GetRobotDevSuiteMetadata(), robotDevSuiteQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.RobotDevSuiteStatus = robotv1alpha1.RobotDevSuiteInstanceStatus{}
	} else if err != nil {
		return err
	} else {

		if instance.Spec.RobotDevSuiteTemplate.IDEEnabled || instance.Spec.RobotDevSuiteTemplate.VDIEnabled || instance.Spec.RobotDevSuiteTemplate.RemoteIDEEnabled {

			if !reflect.DeepEqual(instance.Spec.RobotDevSuiteTemplate, robotDevSuiteQuery.Spec) {
				robotDevSuiteQuery.Spec = instance.Spec.RobotDevSuiteTemplate
				err = r.Update(ctx, robotDevSuiteQuery)
				if err != nil {
					return err
				}
			}

			instance.Status.RobotDevSuiteStatus.Resource.Created = true
			reference.SetReference(&instance.Status.RobotDevSuiteStatus.Resource.Reference, robotDevSuiteQuery.TypeMeta, robotDevSuiteQuery.ObjectMeta)
			instance.Status.RobotDevSuiteStatus.Status = robotDevSuiteQuery.Status

		} else {

			err := r.Delete(ctx, robotDevSuiteQuery)
			if err != nil {
				return err
			}

		}

	}

	return nil
}

func (r *RobotReconciler) reconcileCheckWorkspaceManager(ctx context.Context, instance *robotv1alpha1.Robot) error {

	workspaceManagerQuery := &robotv1alpha1.WorkspaceManager{}
	err := r.Get(ctx, *instance.GetWorkspaceManagerMetadata(), workspaceManagerQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.WorkspaceManagerStatus = robotv1alpha1.WorkspaceManagerInstanceStatus{}
	} else if err != nil {
		return err
	} else {

		instance.Status.WorkspaceManagerStatus.Resource.Created = true
		reference.SetReference(&instance.Status.WorkspaceManagerStatus.Resource.Reference, workspaceManagerQuery.TypeMeta, workspaceManagerQuery.ObjectMeta)
		instance.Status.WorkspaceManagerStatus.Status = workspaceManagerQuery.Status

		if !reflect.DeepEqual(instance.Spec.WorkspaceManagerTemplate.Workspaces, workspaceManagerQuery.Spec.Workspaces) {
			workspaceManagerQuery.Spec = instance.Spec.WorkspaceManagerTemplate
			workspaceManagerQuery.Spec.UpdateNeeded = true
			err = r.Update(ctx, workspaceManagerQuery)
			if err != nil {
				return err
			}

			// set phase configuring
			instance.Status.WorkspaceManagerStatus.Resource.Created = true
			instance.Status.WorkspaceManagerStatus.Status = robotv1alpha1.WorkspaceManagerStatus{}
			instance.Status.WorkspaceManagerStatus.Status.Phase = robotv1alpha1.WorkspaceManagerPhaseConfiguringWorkspaces
		}

	}

	return nil
}

func (r *RobotReconciler) reconcileCheckAttachedBuildManager(ctx context.Context, instance *robotv1alpha1.Robot) error {

	bmReference := instance.Status.AttachedBuildObject.Reference

	// If any build object was attached, record it's status
	if bmReference.Name != "" {

		buildManager := &robotv1alpha1.BuildManager{}
		err := r.Get(ctx, types.NamespacedName{Namespace: bmReference.Namespace, Name: bmReference.Name}, buildManager)
		if err != nil && errors.IsNotFound(err) {
			// TODO: Empty the reference fields
			return err
		} else if err != nil {
			return err
		} else {
			instance.Status.AttachedBuildObject.Status = buildManager.Status
		}

	}

	return nil
}

func (r *RobotReconciler) reconcileCheckAttachedLaunchManager(ctx context.Context, instance *robotv1alpha1.Robot) error {

	for k, lm := range instance.Status.AttachedLaunchObjects {
		launchManager := &robotv1alpha1.LaunchManager{}
		err := r.Get(ctx, types.NamespacedName{Namespace: lm.Reference.Namespace, Name: lm.Reference.Name}, launchManager)
		if err != nil && errors.IsNotFound(err) {
			// TODO: Empty the reference fields
			return err
		} else if err != nil {
			return err
		} else {
			lm.Status = launchManager.Status
		}

		instance.Status.AttachedLaunchObjects[k] = lm
	}

	return nil
}
