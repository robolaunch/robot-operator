package code_editor

import (
	"context"

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

		// make deployment dynamic

		instance.Status.DeploymentStatus.Resource.Created = true
		reference.SetReference(&instance.Status.DeploymentStatus.Resource.Reference, deploymentQuery.TypeMeta, deploymentQuery.ObjectMeta)
	}

	return nil
}
