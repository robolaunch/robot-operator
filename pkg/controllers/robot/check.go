package robot

import (
	"context"

	robotv1alpha1 "github.com/robolaunch/robot-operator/api/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
)

func (r *RobotReconciler) reconcileCheckPVCs(ctx context.Context, instance *robotv1alpha1.Robot) error {

	pvcVarQuery := &corev1.PersistentVolumeClaim{}
	err := r.Get(ctx, *instance.GetPVCVarMetadata(), pvcVarQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.VolumeStatus.Var = false
	} else if err != nil {
		return err
	}

	pvcOptQuery := &corev1.PersistentVolumeClaim{}
	err = r.Get(ctx, *instance.GetPVCOptMetadata(), pvcOptQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.VolumeStatus.Opt = false
	} else if err != nil {
		return err
	}

	pvcEtcQuery := &corev1.PersistentVolumeClaim{}
	err = r.Get(ctx, *instance.GetPVCEtcMetadata(), pvcEtcQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.VolumeStatus.Etc = false
	} else if err != nil {
		return err
	}

	pvcUsrQuery := &corev1.PersistentVolumeClaim{}
	err = r.Get(ctx, *instance.GetPVCUsrMetadata(), pvcUsrQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.VolumeStatus.Usr = false
	} else if err != nil {
		return err
	}

	pvcDisplayQuery := &corev1.PersistentVolumeClaim{}
	err = r.Get(ctx, *instance.GetPVCDisplayMetadata(), pvcDisplayQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.VolumeStatus.Display = false
	} else if err != nil {
		return err
	}

	pvcWorkspaceQuery := &corev1.PersistentVolumeClaim{}
	err = r.Get(ctx, *instance.GetPVCWorkspaceMetadata(), pvcWorkspaceQuery)
	if err != nil && errors.IsNotFound(err) {
		instance.Status.VolumeStatus.Workspace = false
	} else if err != nil {
		return err
	}

	return nil
}
