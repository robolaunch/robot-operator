package code_editor

import (
	"context"
	"reflect"

	"github.com/robolaunch/robot-operator/internal"
	"github.com/robolaunch/robot-operator/internal/label"
	"github.com/robolaunch/robot-operator/internal/platform"
	"github.com/robolaunch/robot-operator/internal/reference"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	appsv1 "k8s.io/api/apps/v1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/types"
)

func (r *CodeEditorReconciler) reconcileCheckPVCs(ctx context.Context, instance *robotv1alpha2.CodeEditor) error {

	for key, pvcStatus := range instance.Status.PVCStatuses {

		pvcQuery := &corev1.PersistentVolumeClaim{}
		err := r.Get(ctx, *instance.GetPersistentVolumeClaimMetadata(key), pvcQuery)
		if err != nil && errors.IsNotFound(err) {
			pvcStatus.Resource.Created = false
		} else if err != nil {
			return err
		} else {
			pvcStatus.Resource.Created = true
			reference.SetReference(&pvcStatus.Resource.Reference, pvcQuery.TypeMeta, pvcQuery.ObjectMeta)
			pvcStatus.Status = pvcQuery.Status
		}

		instance.Status.PVCStatuses[key] = pvcStatus

	}

	return nil
}

func (r *CodeEditorReconciler) reconcileCheckExternalVolumes(ctx context.Context, instance *robotv1alpha2.CodeEditor) error {

	for key, evStatus := range instance.Status.ExternalVolumeStatuses {

		pvcQuery := &corev1.PersistentVolumeClaim{}
		err := r.Get(ctx, types.NamespacedName{
			Namespace: instance.Namespace,
			Name:      evStatus.Name,
		}, pvcQuery)
		if err != nil && errors.IsNotFound(err) {
			evStatus.Exists = false
		} else if err != nil {
			return err
		} else {
			evStatus.Exists = true
		}

		instance.Status.ExternalVolumeStatuses[key] = evStatus

	}

	return nil
}

func (r *CodeEditorReconciler) reconcileCheckDeployment(ctx context.Context, instance *robotv1alpha2.CodeEditor) error {

	deploymentQuery := &appsv1.Deployment{}
	err := r.Get(ctx, *instance.GetDeploymentMetadata(), deploymentQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.DeploymentStatus = robotv1alpha2.OwnedDeploymentStatus{}
	} else if err != nil {
		return err
	} else {

		platformMeta := label.GetPlatformMeta(instance)

		desiredImage, err := platform.GetToolsImage(instance, platformMeta.Version, internal.CODE_EDITOR_APP_NAME, instance.Spec.Version)
		if err != nil {
			return err
		}

		actualImage := deploymentQuery.Spec.Template.Spec.Containers[0].Image

		remoteConfigSynced := (instance.Spec.Remote && reflect.DeepEqual(deploymentQuery.Spec.Template.Spec.Hostname, instance.Name) && reflect.DeepEqual(deploymentQuery.Spec.Template.Spec.Subdomain, instance.Name)) ||
			(!instance.Spec.Remote && reflect.DeepEqual(deploymentQuery.Spec.Template.Spec.Hostname, "") && reflect.DeepEqual(deploymentQuery.Spec.Template.Spec.Subdomain, ""))

		if !reflect.DeepEqual(desiredImage, actualImage) ||
			!remoteConfigSynced {
			err := r.updateDeployment(ctx, instance)
			if err != nil {
				return err
			}
		}

		instance.Status.DeploymentStatus.Resource.Created = true
		reference.SetReference(&instance.Status.DeploymentStatus.Resource.Reference, deploymentQuery.TypeMeta, deploymentQuery.ObjectMeta)
	}

	return nil
}

func (r *CodeEditorReconciler) reconcileCheckService(ctx context.Context, instance *robotv1alpha2.CodeEditor) error {

	serviceQuery := &corev1.Service{}
	err := r.Get(ctx, *instance.GetServiceMetadata(), serviceQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.ServiceStatus = robotv1alpha2.OwnedServiceStatus{}
	} else if err != nil {
		return err
	} else {

		// make service dynamic

		instance.Status.ServiceStatus.Resource.Created = true
		reference.SetReference(&instance.Status.ServiceStatus.Resource.Reference, serviceQuery.TypeMeta, serviceQuery.ObjectMeta)
	}

	return nil
}
