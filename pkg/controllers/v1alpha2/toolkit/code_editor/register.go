package code_editor

import (
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	corev1 "k8s.io/api/core/v1"
)

func (r *CodeEditorReconciler) registerPVCs(instance *robotv1alpha2.CodeEditor) {

	pvcStatuses := []robotv1alpha2.OwnedPVCStatus{}

	if len(instance.Spec.VolumeClaimTemplates) != len(instance.Status.PVCStatuses) {
		for key := range instance.Spec.VolumeClaimTemplates {
			pvcStatus := robotv1alpha2.OwnedPVCStatus{
				Resource: robotv1alpha2.OwnedResourceStatus{
					Reference: corev1.ObjectReference{
						Namespace: instance.Namespace,
						Name:      instance.GetPersistentVolumeClaimMetadata(key).Name,
					},
				},
			}
			pvcStatuses = append(pvcStatuses, pvcStatus)
		}
		instance.Status.PVCStatuses = pvcStatuses
	}
}

func (r *CodeEditorReconciler) registerExternalVolumes(instance *robotv1alpha2.CodeEditor) {

	evStatuses := []robotv1alpha2.ExternalVolumeStatus{}

	if len(instance.Spec.ExternalVolumes) != len(instance.Status.ExternalVolumeStatuses) {
		for _, ev := range instance.Spec.ExternalVolumes {
			evStatus := robotv1alpha2.ExternalVolumeStatus{
				Name: ev.Name,
			}
			evStatuses = append(evStatuses, evStatus)
		}
		instance.Status.ExternalVolumeStatuses = evStatuses
	}
}
