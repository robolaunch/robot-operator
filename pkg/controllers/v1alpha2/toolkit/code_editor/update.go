package code_editor

import (
	"context"

	v1alpha2_resources "github.com/robolaunch/robot-operator/internal/resources/v1alpha2"
	robotv1alpha2 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha2"
	"k8s.io/apimachinery/pkg/api/errors"
	ctrl "sigs.k8s.io/controller-runtime"
)

func (r *CodeEditorReconciler) updateDeployment(ctx context.Context, instance *robotv1alpha2.CodeEditor) error {

	node, err := r.reconcileGetNode(ctx, instance)
	if err != nil {
		return err
	}

	deployment := v1alpha2_resources.GetCodeEditorDeployment(instance, instance.GetDeploymentMetadata(), *node)

	err = ctrl.SetControllerReference(instance, deployment, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Update(ctx, deployment)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Deployment " + deployment.Name + " is updated.")
	return nil
}

func (r *CodeEditorReconciler) updateService(ctx context.Context, instance *robotv1alpha2.CodeEditor) error {

	service := v1alpha2_resources.GetCodeEditorService(instance, instance.GetServiceMetadata())

	err := ctrl.SetControllerReference(instance, service, r.Scheme)
	if err != nil {
		return err
	}

	err = r.Update(ctx, service)
	if err != nil && errors.IsAlreadyExists(err) {
		return nil
	} else if err != nil {
		return err
	}

	logger.Info("STATUS: Service " + service.Name + " is updated.")
	return nil
}
