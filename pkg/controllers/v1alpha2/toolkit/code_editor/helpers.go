package code_editor

import (
	"context"

	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/client-go/util/retry"
)

func (r *CodeEditorReconciler) reconcileGetInstance(ctx context.Context, meta types.NamespacedName) (*robotv1alpha2.CodeEditor, error) {
	instance := &robotv1alpha2.CodeEditor{}
	err := r.Get(ctx, meta, instance)
	if err != nil {
		return &robotv1alpha2.CodeEditor{}, err
	}

	return instance, nil
}

func (r *CodeEditorReconciler) reconcileUpdateInstanceStatus(ctx context.Context, instance *robotv1alpha2.CodeEditor) error {
	return retry.RetryOnConflict(retry.DefaultRetry, func() error {
		instanceLV := &robotv1alpha2.CodeEditor{}
		err := r.Get(ctx, types.NamespacedName{
			Name:      instance.Name,
			Namespace: instance.Namespace,
		}, instanceLV)

		if err == nil {
			instance.ResourceVersion = instanceLV.ResourceVersion
		}

		err1 := r.Status().Update(ctx, instance)
		return err1
	})
}
