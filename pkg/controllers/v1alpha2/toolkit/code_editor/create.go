package code_editor

import (
	"context"

	v1alpha2_resources "github.com/robolaunch/robot-operator/internal/resources/v1alpha2"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	"k8s.io/apimachinery/pkg/api/errors"
	ctrl "sigs.k8s.io/controller-runtime"
)

func (r *CodeEditorReconciler) createPersistentVolumeClaim(ctx context.Context, instance *robotv1alpha2.CodeEditor, key int) error {

	pvc := v1alpha2_resources.GetCodeEditorPersistentVolumeClaim(instance, instance.GetPersistentVolumeClaimMetadata(key), key)

	err := ctrl.SetControllerReference(instance, pvc, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Create(ctx, pvc)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: PVC " + instance.GetPersistentVolumeClaimMetadata(key).Name + " is created.")
	return nil
}
