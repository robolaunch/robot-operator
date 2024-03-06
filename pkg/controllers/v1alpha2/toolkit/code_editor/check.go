package code_editor

import (
	"context"

	"github.com/robolaunch/robot-operator/internal/reference"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
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
