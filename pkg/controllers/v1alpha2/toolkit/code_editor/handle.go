package code_editor

import (
	"context"

	robotErr "github.com/robolaunch/robot-operator/internal/error"

	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
)

func (r *CodeEditorReconciler) reconcileHandlePVCs(ctx context.Context, instance *robotv1alpha2.CodeEditor) error {

	for key, pvcStatus := range instance.Status.PVCStatuses {
		if !pvcStatus.Resource.Created {

			instance.Status.Phase = robotv1alpha2.CodeEditorPhaseCreatingPVCs
			err := r.createPersistentVolumeClaim(ctx, instance, key)
			if err != nil {
				return err
			}

			pvcStatus.Resource.Created = true
			instance.Status.PVCStatuses[key] = pvcStatus

			return &robotErr.CreatingResourceError{
				ResourceKind:      "PersistentVolumeClaim",
				ResourceName:      instance.GetPersistentVolumeClaimMetadata(key).Name,
				ResourceNamespace: instance.GetPersistentVolumeClaimMetadata(key).Namespace,
			}
		}
	}

	return nil
}
